//! USB CDC (Communications Device Class) wrapper.
//!
//! This module provides a Rust interface to the Flipper Zero's USB CDC serial port,
//! allowing apps to communicate with a host computer over USB as a virtual serial device.

use core::ffi::c_void;
use core::num::NonZero;
use core::ptr;
use core::sync::atomic::{AtomicBool, AtomicPtr, Ordering};

use crate::furi::stream_buffer::StreamBuffer;
use crate::furi::thread::{self, ThreadId};
use crate::furi::time::FuriDuration;
use crate::{debug, trace};
use flipperzero_sys::{self as sys};
use sys::furi::FuriBox;

mod tx_progress;
use tx_progress::TxProgress;

/// USB CDC operating mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum CdcMode {
    /// Single CDC port. App owns channel 0. Flipper CLI is unavailable.
    Single,
    /// Dual CDC port. Channel 0 = firmware CLI, channel 1 = app data.
    Dual,
}

/// USB CDC errors.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// USB mode switch is locked by another component.
    UsbLocked,
    /// No active DTR session, or the USB session suspended. Recreate after suspend.
    Disconnected,
    /// Completion was not observed before the deadline; delivery is unknown.
    /// Pending endpoint state is retained and cannot be overwritten by another TX.
    CompletionTimeout,
}

// The firmware updates its cached DTR/RTS state only when a non-null callback
// table is installed. Keep one installed even for synchronous-only sessions.
// Static storage survives enumeration and every synchronous method call.
static IDLE_CALLBACKS: sys::CdcCallbacks = sys::CdcCallbacks {
    tx_ep_callback: Some(idle_tx_callback),
    rx_ep_callback: None,
    state_callback: Some(idle_state_callback),
    ctrl_line_callback: None,
    config_callback: None,
};

/// RAII handle to a USB CDC interface.
///
/// Saves the previous USB configuration on creation and restores it on drop.
/// While active, the USB mode is locked to prevent other code from switching it.
pub struct UsbCdc {
    if_num: u8,
    prev_config: *mut sys::FuriHalUsbInterface,
    tx_state: FuriBox<TxState>,
}

struct TxState {
    progress: TxProgress,
    changing_callbacks: AtomicBool,
    if_num: u8,
}

impl UsbCdc {
    /// Enable USB CDC mode.
    ///
    /// Switches the Flipper's USB interface to CDC mode and locks it. The previous
    /// USB configuration is saved and will be restored when this handle is dropped.
    ///
    /// Returns [`Error::UsbLocked`] if the USB mode is already locked by another component.
    ///
    /// # Safety
    ///
    /// The caller must exclusively coordinate USB configuration with all other firmware
    /// components for the entire handle lifetime. HAL's check/switch/lock operations are
    /// separate calls, not an atomic ownership claim. The selected CDC channel must have
    /// no existing callbacks, pending endpoint transfers, or users (in single mode,
    /// explicitly stop the firmware CLI first). The previous interface must support restoration with a null context;
    /// HAL exposes no getter for its original context. The caller must restore any CLI
    /// session it stopped after dropping this handle. Do not unload the application
    /// while a leaked handle or receiver is still active.
    pub unsafe fn enable(mode: CdcMode) -> Result<Self, Error> {
        unsafe {
            if sys::furi_hal_usb_is_locked() {
                return Err(Error::UsbLocked);
            }

            let prev_config = sys::furi_hal_usb_get_config();

            let (usb_if, if_num) = match mode {
                CdcMode::Single => (&raw mut sys::usb_cdc_single, 0),
                CdcMode::Dual => (&raw mut sys::usb_cdc_dual, 1),
            };

            let mut tx_state = FuriBox::new(TxState {
                progress: TxProgress::new(prev_config != usb_if),
                changing_callbacks: AtomicBool::new(false),
                if_num,
            });
            let tx_ptr = FuriBox::as_mut_ptr(&mut tx_state);
            // Install before enumeration: SET_CONTROL_LINE_STATE can arrive as
            // soon as the host opens the newly exposed interface. The constructor
            // contract guarantees this channel has no previous callback owner.
            set_callbacks(
                if_num,
                (&raw const IDLE_CALLBACKS).cast_mut(),
                tx_ptr.cast(),
                tx_ptr,
            );
            if !sys::furi_hal_usb_set_config(usb_if, ptr::null_mut()) {
                set_callbacks(if_num, ptr::null_mut(), ptr::null_mut(), tx_ptr);
                return Err(Error::UsbLocked);
            }

            sys::furi_hal_usb_lock();

            Ok(UsbCdc {
                if_num,
                prev_config,
                tx_state,
            })
        }
    }

    /// Send at most one 64-byte packet and wait for endpoint completion.
    ///
    /// Full packets receive a zero-length terminator after their data completion.
    /// Success includes that terminator's completion, not host application receipt.
    /// The timeout bounds waiting for both a previous pending transfer and this one.
    /// A timeout leaves delivery unknown and retains pending state. A suspended USB
    /// session is terminal: recreate it instead of treating reconnect as completion.
    /// Call from thread context with interrupts and the scheduler enabled.
    ///
    /// # Panics
    ///
    /// Panics if `data.len()` exceeds 64 bytes.
    pub fn tx(&self, data: &[u8], timeout: FuriDuration) -> Result<(), Error> {
        assert!(
            data.len() <= CDC_WORKER_BUFFER_LEN,
            "CDC packet exceeds 64 bytes"
        );
        let start = unsafe { sys::furi_get_tick() };
        self.wait_tx(start, timeout)?;
        if !self.ctrl_line_state().dtr() {
            return Err(Error::Disconnected);
        }
        // Exclude completion/suspend while claiming the endpoint and submitting.
        // HAL copies the payload synchronously into packet memory; no slice is retained.
        unsafe {
            let critical = sys::__furi_critical_enter();
            let claimed = self
                .tx_state
                .progress
                .begin(data.len() == CDC_WORKER_BUFFER_LEN);
            if claimed {
                sys::furi_hal_cdc_send(self.if_num, data.as_ptr().cast_mut(), data.len() as u16);
            }
            sys::__furi_critical_exit(critical);
            if !claimed {
                return Err(Error::Disconnected);
            }
        }
        self.wait_tx(start, timeout)
    }

    fn wait_tx(&self, start: u32, timeout: FuriDuration) -> Result<(), Error> {
        loop {
            if self.tx_state.progress.disconnected() {
                return Err(Error::Disconnected);
            }
            if self.tx_state.progress.idle() {
                return Ok(());
            }
            if unsafe { sys::furi_get_tick() }.wrapping_sub(start) >= timeout.as_ticks() {
                return Err(Error::CompletionTimeout);
            }
            unsafe { sys::furi_delay_tick(1) };
        }
    }

    /// Receive data from USB CDC.
    ///
    /// Returns the number of bytes actually read into `buf`.
    ///
    /// # Panics
    ///
    /// Panics if `buf.len()` exceeds `u16::MAX`.
    pub fn rx(&self, buf: &mut [u8]) -> usize {
        let max_len: u16 = buf
            .len()
            .try_into()
            .expect("buffer length exceeds u16::MAX");
        let received = unsafe { sys::furi_hal_cdc_receive(self.if_num, buf.as_mut_ptr(), max_len) };
        // furi_hal_cdc_receive returns i32; negative means no data
        if received < 0 { 0 } else { received as usize }
    }

    /// Get the current line coding (baud rate, parity, etc.) set by the host.
    pub fn line_coding(&self) -> LineCoding {
        // USB control requests update this packed static value from interrupt context.
        // Exclude those writes while copying, not merely unaligned access.
        unsafe {
            let critical = sys::__furi_critical_enter();
            let result = LineCoding::from_raw(sys::furi_hal_cdc_get_port_settings(self.if_num));
            sys::__furi_critical_exit(critical);
            result
        }
    }

    /// Get the current control line state (DTR/RTS) set by the host.
    pub fn ctrl_line_state(&self) -> CtrlLineState {
        unsafe {
            let critical = sys::__furi_critical_enter();
            let state = sys::furi_hal_cdc_get_ctrl_line_state(self.if_num);
            sys::__furi_critical_exit(critical);
            CtrlLineState(state)
        }
    }

    /// Create an async receiver for incoming USB CDC data.
    ///
    /// The provided callback will be invoked with received data on a dedicated worker thread.
    /// The receiver is active until dropped and takes ownership of this handle. This
    /// prevents concurrent receivers and synchronous reads competing for the endpoint.
    /// Captures must be owned (`Send + 'static`) because forgetting a receiver must
    /// not leave the worker accessing expired stack references.
    ///
    /// The callback must return promptly so dropping the receiver can join the worker.
    /// The 2048-byte queue is bounded; excess input is currently discarded.
    pub fn async_receiver<F: FnMut(&[u8]) + Send + 'static>(self, on_rx: F) -> AsyncCdcReceiver<F> {
        AsyncCdcReceiver::new(self, on_rx)
    }
}

impl Drop for UsbCdc {
    fn drop(&mut self) {
        // Clear CDC callbacks so they no longer reference any context.
        unsafe {
            set_callbacks(
                self.if_num,
                ptr::null_mut(),
                ptr::null_mut(),
                FuriBox::as_ptr(&self.tx_state),
            )
        };

        // Unlock and restore the previous USB configuration.
        unsafe {
            sys::furi_hal_usb_unlock();
            sys::furi_hal_usb_set_config(self.prev_config, ptr::null_mut());
        }
    }
}

/// USB CDC line coding parameters set by the host.
#[derive(Debug, Clone, Copy)]
pub struct LineCoding {
    /// Baud rate in bits per second.
    pub baud_rate: u32,
    /// Stop bits.
    pub stop_bits: StopBits,
    /// Parity type.
    pub parity: Parity,
    /// Data bits.
    pub data_bits: DataBits,
}

impl LineCoding {
    /// # Safety
    ///
    /// `ptr` must point to a valid `usb_cdc_line_coding` struct.
    unsafe fn from_raw(ptr: *const sys::usb_cdc_line_coding) -> Self {
        // Use read_unaligned because usb_cdc_line_coding is repr(C, packed).
        let raw = unsafe { ptr.read_unaligned() };
        LineCoding {
            baud_rate: raw.dwDTERate,
            stop_bits: StopBits::from_raw(raw.bCharFormat),
            parity: Parity::from_raw(raw.bParityType),
            data_bits: DataBits::from_raw(raw.bDataBits),
        }
    }
}

/// Stop bits setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StopBits {
    One,
    OneAndHalf,
    Two,
    /// Unknown value from the host.
    Unknown(u8),
}

impl StopBits {
    fn from_raw(val: u8) -> Self {
        match val {
            0 => StopBits::One,
            1 => StopBits::OneAndHalf,
            2 => StopBits::Two,
            v => StopBits::Unknown(v),
        }
    }
}

/// Parity setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Parity {
    None,
    Odd,
    Even,
    Mark,
    Space,
    /// Unknown value from the host.
    Unknown(u8),
}

impl Parity {
    fn from_raw(val: u8) -> Self {
        match val {
            0 => Parity::None,
            1 => Parity::Odd,
            2 => Parity::Even,
            3 => Parity::Mark,
            4 => Parity::Space,
            v => Parity::Unknown(v),
        }
    }
}

/// Data bits setting.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DataBits {
    Five,
    Six,
    Seven,
    Eight,
    Sixteen,
    /// Unknown value from the host.
    Unknown(u8),
}

impl DataBits {
    fn from_raw(val: u8) -> Self {
        match val {
            5 => DataBits::Five,
            6 => DataBits::Six,
            7 => DataBits::Seven,
            8 => DataBits::Eight,
            16 => DataBits::Sixteen,
            v => DataBits::Unknown(v),
        }
    }
}

/// Control line state from the host (DTR/RTS).
#[derive(Debug, Clone, Copy)]
pub struct CtrlLineState(u8);

impl CtrlLineState {
    /// Data Terminal Ready signal.
    pub fn dtr(&self) -> bool {
        self.0 & sys::CdcCtrlLineDTR.0 != 0
    }

    /// Request To Send signal.
    pub fn rts(&self) -> bool {
        self.0 & sys::CdcCtrlLineRTS.0 != 0
    }
}

// --- Async receiver ---

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct WorkerEvent(u32);

impl WorkerEvent {
    /// Stop worker.
    pub const FLAG_STOP: u32 = 1 << 0;
    /// New data available.
    pub const FLAG_DATA: u32 = 1 << 1;

    /// Mask of all supported events.
    pub const MASK: u32 = Self::FLAG_STOP | Self::FLAG_DATA;

    pub fn is_stop(self) -> bool {
        self.0 & Self::FLAG_STOP != 0
    }

    pub fn is_rx_data(self) -> bool {
        self.0 & Self::FLAG_DATA != 0
    }
}

/// Async receiver that dispatches incoming USB CDC data via callback.
///
/// This spawns a dedicated worker thread to dispatch received data.
pub struct AsyncCdcReceiver<F>
where
    F: FnMut(&[u8]) + Send + 'static,
{
    usb_cdc: UsbCdc,
    context: FuriBox<Context<F>>,
}

struct Context<F: FnMut(&[u8]) + Send + 'static> {
    rx_stream: StreamBuffer,
    on_rx: F,
    worker_thread: AtomicPtr<sys::FuriThread>,
    if_num: u8,
    cdc_callbacks: sys::CdcCallbacks,
    tx_state: *const TxState,
}

impl<F> AsyncCdcReceiver<F>
where
    F: FnMut(&[u8]) + Send + 'static,
{
    /// Submit one endpoint packet while async reception is active.
    ///
    /// This has the same size and delivery limitations as [`UsbCdc::tx`].
    pub fn tx(&self, data: &[u8], timeout: FuriDuration) -> Result<(), Error> {
        self.usb_cdc.tx(data, timeout)
    }

    fn new(usb_cdc: UsbCdc, on_rx: F) -> Self {
        let rx_stream = StreamBuffer::new(NonZero::new(2048).unwrap(), 1);

        let mut context = FuriBox::new(Context {
            rx_stream,
            on_rx,
            worker_thread: AtomicPtr::new(ptr::null_mut()),
            if_num: usb_cdc.if_num,
            tx_state: FuriBox::as_ptr(&usb_cdc.tx_state),
            cdc_callbacks: sys::CdcCallbacks {
                tx_ep_callback: Some(async_tx_callback::<F>),
                rx_ep_callback: Some(cdc_rx_callback::<F>),
                state_callback: Some(async_state_callback::<F>),
                ctrl_line_callback: None,
                config_callback: None,
            },
        });

        unsafe {
            // SAFETY: Grabbing the context pointer with `as_mut_ptr` is fine,
            // since it doesn't create an intermediate reference.
            let worker_thread = sys::furi_thread_alloc_ex(
                c"AsyncCdcReceiverWorker".as_ptr(),
                1024,
                Some(async_cdc_receiver_worker::<F>),
                FuriBox::as_mut_ptr(&mut context) as *mut _,
            );

            // SAFETY: Since thread hasn't started yet, it's still safe to reference `Context`.
            context
                .worker_thread
                .store(worker_thread, Ordering::Release);

            // SAFETY: From this point on we must carefully respect the aliasing rules.
            sys::furi_thread_start(worker_thread);

            // SAFETY: Grabbing the context pointer with `as_mut_ptr` is fine,
            // since it doesn't create an intermediate reference. We pass a pointer to
            // the `cdc_callbacks` field inside the FuriBox-allocated Context.
            let context_ptr = FuriBox::as_mut_ptr(&mut context);
            set_callbacks(
                usb_cdc.if_num,
                &raw mut (*context_ptr).cdc_callbacks,
                context_ptr.cast(),
                FuriBox::as_ptr(&usb_cdc.tx_state),
            );
        }

        AsyncCdcReceiver { usb_cdc, context }
    }
}

impl<F: FnMut(&[u8]) + Send + 'static> Drop for AsyncCdcReceiver<F> {
    fn drop(&mut self) {
        // Clear the callback so it no longer references `Context`.
        unsafe {
            set_callbacks(
                self.usb_cdc.if_num,
                ptr::null_mut(),
                ptr::null_mut(),
                FuriBox::as_ptr(&self.usb_cdc.tx_state),
            );
        }

        // SAFETY: Worker thread is still running, so be careful not to create a reference to `Context`.
        // Using `as_mut_ptr` is fine since it only creates a reference to the `Box` not the `Context` inside.
        let context = FuriBox::as_mut_ptr(&mut self.context);
        let worker_thread = unsafe { (*context).worker_thread.load(Ordering::Acquire) };

        if !worker_thread.is_null() {
            let thread_id = unsafe { ThreadId::from_furi_thread(worker_thread) };
            thread::set_flags(thread_id, WorkerEvent::FLAG_STOP).unwrap();

            unsafe {
                sys::furi_thread_join(worker_thread);
                (*context)
                    .worker_thread
                    .store(ptr::null_mut(), Ordering::Release);
                sys::furi_thread_free(worker_thread);
            }
        }
    }
}

const CDC_WORKER_BUFFER_LEN: usize = 64;

/// CDC rx_ep_callback — may run in USB interrupt context.
///
/// Pulls available data from the CDC endpoint and writes it into the stream buffer,
/// then signals the worker thread.
unsafe extern "C" fn cdc_rx_callback<F: FnMut(&[u8]) + Send + 'static>(context: *mut c_void) {
    let context = context.cast_const() as *const Context<F>;

    // Pull data from the CDC endpoint.
    let mut buf = [0u8; CDC_WORKER_BUFFER_LEN];
    let if_num = unsafe { (*context).if_num };
    let received = unsafe {
        sys::furi_hal_cdc_receive(if_num, buf.as_mut_ptr(), CDC_WORKER_BUFFER_LEN as u16)
    };

    if received > 0 {
        let data = &buf[..received as usize];
        unsafe { (*context).rx_stream.send(data, FuriDuration::ZERO) };

        let worker_thread = unsafe { (*context).worker_thread.load(Ordering::Acquire) };
        if !worker_thread.is_null() {
            let thread_id = unsafe { ThreadId::from_furi_thread(worker_thread) };
            thread::set_flags(thread_id, WorkerEvent::FLAG_DATA).unwrap();
        }
    }
}

unsafe extern "C" fn async_cdc_receiver_worker<F: FnMut(&[u8]) + Send + 'static>(
    context: *mut c_void,
) -> i32 {
    debug!("Starting CDC async worker");
    assert!(!context.is_null());
    let context = context.cast::<Context<F>>();

    loop {
        let events = WorkerEvent(
            thread::wait_any_flags(WorkerEvent::MASK, true, FuriDuration::MAX).unwrap_or(0),
        );
        trace!("CDC WorkerEvent: {}", events.0);

        if events.is_stop() {
            debug!("Stopping CDC async worker");
            break;
        }

        if events.is_rx_data() {
            loop {
                // Observe STOP between callbacks without clearing DATA. A new DATA
                // notification racing the final empty read remains pending for the
                // outer wait. Drop detaches the producer first, but need not wait
                // for the whole queued backlog to be delivered before joining.
                if WorkerEvent(thread::get_flags().unwrap_or(0)).is_stop() {
                    return 0;
                }
                let mut data = [0u8; CDC_WORKER_BUFFER_LEN];
                let len = unsafe { (*context).rx_stream.receive(&mut data, FuriDuration::ZERO) };

                if len == 0 {
                    break;
                }

                unsafe { ((*context).on_rx)(&data[..len]) }
            }
        }
    }

    0
}

// SAFETY: Only our callbacks (or null) occupy the exclusively owned channel.
// Our state callback performs atomic operations only, so the HAL's synchronous
// notification cannot block inside the critical section. Callback replacement
// invokes synthetic disconnect/connect notifications; suppress only those while
// USB interrupts and task switches are excluded. Actual suspend makes TX terminal.
// Detach returns after any prior ISR callback has finished, before storage is freed.
unsafe fn set_callbacks(
    if_num: u8,
    callbacks: *mut sys::CdcCallbacks,
    context: *mut c_void,
    tx: *const TxState,
) {
    unsafe {
        let critical = sys::__furi_critical_enter();
        (*tx).changing_callbacks.store(true, Ordering::Release);
        sys::furi_hal_cdc_set_callbacks(if_num, callbacks, context);
        (*tx).changing_callbacks.store(false, Ordering::Release);
        sys::__furi_critical_exit(critical);
    }
}

unsafe fn tx_complete(tx: *const TxState) {
    unsafe {
        if (*tx).progress.complete() {
            // The USB driver clears the completion flag before calling us. Its
            // PMA write is synchronous and nonblocking, including from this ISR.
            sys::furi_hal_cdc_send((*tx).if_num, ptr::null_mut(), 0);
        }
    }
}
unsafe fn tx_state_changed(tx: *const TxState, state: sys::CdcState) {
    unsafe {
        if state != sys::CdcStateConnected && !(*tx).changing_callbacks.load(Ordering::Acquire) {
            (*tx).progress.suspend();
        }
    }
}
unsafe extern "C" fn idle_tx_callback(context: *mut c_void) {
    unsafe {
        tx_complete(context.cast());
    }
}
unsafe extern "C" fn idle_state_callback(context: *mut c_void, state: sys::CdcState) {
    unsafe {
        tx_state_changed(context.cast(), state);
    }
}
unsafe extern "C" fn async_tx_callback<F: FnMut(&[u8]) + Send + 'static>(context: *mut c_void) {
    unsafe {
        tx_complete((*context.cast::<Context<F>>()).tx_state);
    }
}
unsafe extern "C" fn async_state_callback<F: FnMut(&[u8]) + Send + 'static>(
    context: *mut c_void,
    state: sys::CdcState,
) {
    unsafe {
        tx_state_changed((*context.cast::<Context<F>>()).tx_state, state);
    }
}
