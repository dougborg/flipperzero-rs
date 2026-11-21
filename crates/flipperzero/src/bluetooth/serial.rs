//! Application-owned BLE serial peripheral.
//!
//! Uses a distinct profile template so firmware RPC cannot replace the application's
//! serial callback. This is not a BLE central/client. The session is polled by its
//! owner; no background Rust closure borrows application data.

use crate::furi::{event_flag::EventFlag, stream_buffer::StreamBuffer, time::FuriDuration};
use core::{
    ffi::c_void,
    num::NonZero,
    ptr,
    sync::atomic::{AtomicPtr, AtomicU32, Ordering},
};
use flipperzero_sys::{
    self as sys,
    furi::{FuriBox, UnsafeRecord},
};

const CAPACITY: u16 = 2048;
const SENT: u32 = 1;
const RESET: u32 = 2;
/// Maximum payload accepted by the firmware serial service per indication.
// BLE_SVC_SERIAL_DATA_LEN_MAX in the firmware serial_service.h (SDK 87.1/88.2).
pub const MAX_PACKET_SIZE: usize = 486;

/// Session or transmit error.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    FailedToStartProfile,
    PacketTooLarge,
    TransmitFailed,
    /// Confirmation has not arrived. Do not resend blindly: delivery is unknown.
    ConfirmationTimeout,
    /// The peer requested reset. Drop the session before starting another one.
    ResetRequested,
}

#[repr(C)]
struct Context {
    // Firmware casts this prefix to FuriHalBleProfileBase.
    base: sys::FuriHalBleProfileBase,
    service: AtomicPtr<sys::BleServiceSerial>,
    rx: StreamBuffer,
    events: EventFlag,
    dropped: AtomicU32,
}

static PROFILE: sys::FuriHalBleProfileTemplate = sys::FuriHalBleProfileTemplate {
    start: Some(start_profile),
    stop: Some(stop_profile),
    get_gap_config: Some(gap_config),
};

/// Exclusive application BLE serial session.
///
/// Drop restores the default profile before freeing callback storage. The serial
/// service is owned by this profile rather than the firmware's RPC profile.
/// This handle is neither Send nor Sync and all I/O requires exclusive access.
pub struct BleSerial {
    bt: UnsafeRecord<sys::Bt>,
    context: FuriBox<Context>,
    tx_pending: bool,
}

impl BleSerial {
    /// Start an application-owned serial profile using current Bluetooth settings.
    /// Bluetooth must already be enabled to advertise; settings are not changed.
    ///
    /// # Safety
    ///
    /// The caller must exclusively own the global Bluetooth profile until this
    /// session is dropped. No other profile-changing API (including Bluetooth test
    /// modes or raw firmware calls) may run during this time. The application must
    /// drop the session before unloading its code; forgetting it is not permitted
    /// across application exit. These obligations cannot be enforced by the
    /// firmware's record API, which is not an exclusive profile lease.
    pub unsafe fn start() -> Result<Self, Error> {
        let bt = unsafe { UnsafeRecord::<sys::Bt>::open(c"bt") };
        unsafe { sys::bt_disconnect(bt.as_ptr()) };
        let mut context = FuriBox::new(Context {
            base: sys::FuriHalBleProfileBase { config: &PROFILE },
            service: AtomicPtr::new(ptr::null_mut()),
            rx: StreamBuffer::new(NonZero::new(CAPACITY as usize).unwrap(), 1),
            events: EventFlag::new(),
            dropped: AtomicU32::new(0),
        });
        let profile = unsafe {
            sys::bt_profile_start(
                bt.as_ptr(),
                &PROFILE,
                FuriBox::as_mut_ptr(&mut context).cast(),
            )
        };
        if profile.is_null() {
            // Even failed startup may have touched the radio: restore/join first.
            assert!(
                unsafe { sys::bt_profile_restore_default(bt.as_ptr()) },
                "BLE profile restoration failed"
            );
            return Err(Error::FailedToStartProfile);
        }
        Ok(Self {
            bt,
            context,
            tx_pending: false,
        })
    }

    /// Receive bytes, waiting up to timeout. Zero means no bytes arrived.
    ///
    /// Credits are replenished only when the local queue is drained. A peer which
    /// ignores credits may lose data; inspect dropped_bytes().
    pub fn read(&mut self, bytes: &mut [u8], timeout: FuriDuration) -> Result<usize, Error> {
        if self.reset_requested() {
            return Err(Error::ResetRequested);
        }
        // SAFETY: this owner is the only reader; firmware callback is the writer.
        let count = unsafe { self.context.rx.receive(bytes, timeout) };
        if self.context.rx.bytes_available() == 0 {
            unsafe { sys::ble_svc_serial_notify_buffer_is_empty(self.service()) };
        }
        Ok(count)
    }

    /// Send one indication and wait for its confirmation.
    ///
    /// A timeout leaves delivery unknown. A later call first waits for the old
    /// indication's confirmation; it never submits two indications concurrently.
    /// The timeout applies separately to each wait, not to a wall-clock deadline.
    pub fn tx(&mut self, bytes: &[u8], timeout: FuriDuration) -> Result<(), Error> {
        if bytes.len() > MAX_PACKET_SIZE {
            return Err(Error::PacketTooLarge);
        }
        if self.reset_requested() {
            return Err(Error::ResetRequested);
        }
        self.wait_confirmation(timeout)?;
        if bytes.is_empty() {
            return Ok(());
        }
        let _ = self.context.events.clear(SENT);
        // SAFETY: serial service ultimately calls aci_gatt_update_char_value_ext
        // with a const payload pointer; it does not mutate this borrowed slice.
        self.tx_pending = true;
        if !unsafe {
            sys::ble_svc_serial_update_tx(
                self.service(),
                bytes.as_ptr().cast_mut(),
                bytes.len() as u16,
            )
        } {
            return Err(Error::TransmitFailed);
        }
        self.wait_confirmation(timeout)
    }

    fn wait_confirmation(&mut self, timeout: FuriDuration) -> Result<(), Error> {
        if !self.tx_pending {
            return Ok(());
        }
        let flags = self
            .context
            .events
            .wait_any_flags(SENT | RESET, false, timeout)
            .map_err(|_| Error::ConfirmationTimeout)?;
        if flags & RESET != 0 {
            return Err(Error::ResetRequested);
        }
        let _ = self.context.events.clear(SENT);
        self.tx_pending = false;
        Ok(())
    }

    pub fn reset_requested(&self) -> bool {
        self.context.events.get() & RESET != 0
    }
    /// Bytes rejected by the receive queue (wrapping u32 counter).
    pub fn dropped_bytes(&self) -> u32 {
        self.context.dropped.load(Ordering::Relaxed)
    }
    fn service(&self) -> *mut sys::BleServiceSerial {
        self.context.service.load(Ordering::Acquire)
    }
}

impl Drop for BleSerial {
    fn drop(&mut self) {
        // Synchronously restore the default profile. The firmware
        // stops this service and joins its BLE event thread before returning, so
        // Context remains alive through every callback. Do not clear the callback
        // and free Context independently: the setter is not a quiescence barrier.
        // Fail-stop rather than freeing callback memory if restoration fails.
        assert!(
            unsafe { sys::bt_profile_restore_default(self.bt.as_ptr()) },
            "BLE profile restoration failed"
        );
    }
}

unsafe extern "C" fn gap_config(config: *mut sys::GapConfig, params: *mut c_void) {
    // Reuse firmware defaults for connection/pairing parameters. The distinct
    // PROFILE identity is retained; the firmware RPC check compares templates.
    unsafe {
        ((*sys::ble_profile_serial).get_gap_config.unwrap())(config, params);
        // First byte is the GAP complete-local-name AD type, not name text.
        (*config).adv_name.fill(0);
        for (slot, byte) in (*config).adv_name.iter_mut().zip(b"\x09BLE Serial App") {
            *slot = *byte as core::ffi::c_char;
        }
    };
}

unsafe extern "C" fn start_profile(params: *mut c_void) -> *mut sys::FuriHalBleProfileBase {
    let context = params.cast::<Context>();
    let service = unsafe { sys::ble_svc_serial_start() };
    if service.is_null() {
        return ptr::null_mut();
    }
    unsafe {
        (*context).service.store(service, Ordering::Release);
        // bt_change_profile starts advertising only after this callback returns.
        sys::ble_svc_serial_set_callbacks(service, CAPACITY, Some(on_event), context.cast());
        sys::ble_svc_serial_set_rpc_active(service, true);
        ptr::addr_of_mut!((*context).base)
    }
}

unsafe extern "C" fn stop_profile(base: *mut sys::FuriHalBleProfileBase) {
    let context = base.cast::<Context>();
    // Firmware has stopped advertising/disconnected before invoking profile stop.
    // Keep Context allocated until bt_profile_restore_default returns and joins
    // the event thread. The service owns/unregisters its event dispatcher entry.
    unsafe { sys::ble_svc_serial_stop((*context).service.load(Ordering::Acquire)) };
}

unsafe extern "C" fn on_event(event: sys::SerialServiceEvent, context: *mut c_void) -> u16 {
    let context = context.cast::<Context>();
    match event.event {
        sys::SerialServiceEventTypeDataReceived => {
            // Zero-length firmware events need not have a non-null data pointer.
            if event.data.size != 0 {
                let bytes = unsafe {
                    core::slice::from_raw_parts(event.data.buffer, event.data.size as usize)
                };
                let written = unsafe { (*context).rx.send(bytes, FuriDuration::ZERO) };
                unsafe {
                    (*context)
                        .dropped
                        .fetch_add((bytes.len() - written) as u32, Ordering::Relaxed)
                };
            }
        }
        sys::SerialServiceEventTypeDataSent => {
            let _ = unsafe { (*context).events.set(SENT) };
        }
        sys::SerialServiceEventTypesBleResetRequest => {
            let _ = unsafe { (*context).events.set(RESET) };
        }
        _ => {}
    }
    // Firmware interprets this as remaining capacity, not bytes consumed.
    unsafe { (*context).rx.spaces_available().min(u16::MAX as usize) as u16 }
}
