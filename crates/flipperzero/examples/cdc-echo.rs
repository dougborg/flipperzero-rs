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
    let duration = unsafe { sys::furi_ms_to_ticks(30_000) };
    let mut buffer = [0u8; 64];
    while unsafe { sys::furi_get_tick() }.wrapping_sub(start) < duration {
        let count = cdc.rx(&mut buffer);
        // The host must send one packet at a time, waiting for the exact echo
        // before sending the next. This is not a throughput benchmark.
        if count != 0 && cdc.ctrl_line_state().dtr() {
            cdc.tx(&buffer[..count]);
        }
        unsafe { sys::furi_delay_ms(2) };
    }
    // Exercise allocation, callback handoff, worker delivery, and teardown.
    // The final five seconds receive only; intentionally no echo during this phase.
    RECEIVED.store(0, Ordering::Relaxed);
    let receiver = cdc.async_receiver(|bytes| {
        RECEIVED.fetch_add(bytes.len() as u32, Ordering::Relaxed);
    });
    unsafe { sys::furi_delay_ms(5_000) };
    drop(receiver);
    flipperzero::println!("CDC async bytes: {}", RECEIVED.load(Ordering::Relaxed));
    0
}
