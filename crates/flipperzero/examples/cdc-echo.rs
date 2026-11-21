//! Bounded dual-port CDC smoke test. See docs/cdc.md before running.
#![no_main]
#![no_std]
extern crate flipperzero_rt;

use core::ffi::CStr;
use core::sync::atomic::{AtomicU32, Ordering};
use flipperzero::cdc::{CdcMode, UsbCdc};
use flipperzero_rt::{entry, manifest};
use flipperzero_sys as sys;

manifest!(name = "CDC bounded test", stack_size = 2048);
entry!(main);

static RECEIVED: AtomicU32 = AtomicU32::new(0);

fn main(_args: Option<&CStr>) -> i32 {
    // Refuse interfaces whose original context cannot be recovered.
    if unsafe { sys::furi_hal_usb_get_config() } != &raw mut sys::usb_cdc_single {
        return 1;
    }
    // SAFETY: This diagnostic requires the operator to stop other USB apps and
    // RPC clients first. Channel 1 is unused in the original single-CDC mode.
    // Channel 0 and its CLI callbacks are left intact; restoration needs no ctx.
    let Ok(cdc) = (unsafe { UsbCdc::enable(CdcMode::Dual) }) else {
        return 2;
    };
    let start = unsafe { sys::furi_get_tick() };
    let duration = unsafe { sys::furi_ms_to_ticks(35_000) };
    let mut buffer = [0u8; 64];
    while unsafe { sys::furi_get_tick() }.wrapping_sub(start) < duration {
        let count = cdc.rx(&mut buffer);
        // A reserved one-byte EOT packet switches to async. No test payload is EOT.
        if count == 1 && buffer[0] == 0x04 {
            break;
        }
        // The host must send one packet at a time, waiting for the exact echo
        // before sending the next. This is not a throughput benchmark.
        if count != 0 && cdc.ctrl_line_state().dtr() {
            cdc.tx(&buffer[..count]);
        }
        unsafe { sys::furi_delay_ms(2) };
    }
    // Exercise allocation, callback handoff, worker delivery, and teardown.
    // READY is sent only after callbacks and the worker are installed. Counting
    // bytes (not packets or a delimiter) tolerates arbitrary stream chunking.
    if unsafe { sys::furi_get_tick() }.wrapping_sub(start) >= duration {
        return 0;
    }
    RECEIVED.store(0, Ordering::Relaxed);
    let receiver = cdc.async_receiver(|bytes| {
        RECEIVED.fetch_add(bytes.len() as u32, Ordering::Relaxed);
    });
    receiver.tx(b"CDC-ASYNC-256\n");
    while RECEIVED.load(Ordering::Relaxed) < 256
        && unsafe { sys::furi_get_tick() }.wrapping_sub(start) < duration
    {
        unsafe { sys::furi_delay_ms(2) };
    }
    let count = RECEIVED.load(Ordering::Relaxed);
    if count >= 256 {
        // Host waits for READY before sending exactly 256 bytes, so READY has
        // completed before this response. Host sends nothing after that payload.
        let mut reply = *b"CDC!\0\0\0\0";
        reply[4..].copy_from_slice(&count.to_le_bytes());
        receiver.tx(&reply);
        // Bounded grace for response delivery; this is not completion feedback.
        unsafe { sys::furi_delay_ms(100) };
    }
    drop(receiver);
    0
}
