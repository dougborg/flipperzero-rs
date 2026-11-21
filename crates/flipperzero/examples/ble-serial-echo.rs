//! Bounded application BLE serial echo. Back exits; automatic exit after 60 seconds.
//! Requires a central supporting Flipper serial indications and receive credits.
#![no_main]
#![no_std]
extern crate flipperzero_alloc;
extern crate flipperzero_rt;
use core::{
    ffi::{CStr, c_void},
    ptr,
};
use flipperzero::{
    bluetooth::serial::BleSerial,
    furi::time::{FuriDuration, FuriInstant},
    info,
};
use flipperzero_rt::{entry, manifest};
use flipperzero_sys::{self as sys, furi::UnsafeRecord};
manifest!(name = "BLE Serial Echo", app_version = 2, stack_size = 4096);
entry!(main);
unsafe extern "C" fn draw(canvas: *mut sys::Canvas, _: *mut c_void) {
    unsafe {
        sys::canvas_draw_str(canvas, 4, 14, c"BLE serial echo".as_ptr());
        sys::canvas_draw_str(canvas, 4, 32, c"Back exits / 60s limit".as_ptr());
        sys::canvas_draw_str(canvas, 4, 48, c"App profile, not RPC".as_ptr());
    }
}
fn main(_: Option<&CStr>) -> i32 {
    // SAFETY: this standalone app is sole profile owner and drops the session
    // before returning/unloading code. It invokes no other profile-changing API.
    let mut serial = match unsafe { BleSerial::start() } {
        Ok(serial) => serial,
        Err(_) => return 1,
    };
    let view = unsafe { sys::view_port_alloc() };
    let gui = unsafe { UnsafeRecord::<sys::Gui>::open(c"gui") };
    unsafe {
        sys::view_port_draw_callback_set(view, Some(draw), ptr::null_mut());
        sys::gui_add_view_port(gui.as_ptr(), view, sys::GuiLayerFullscreen);
    }
    let start = FuriInstant::now();
    let mut data = [0; 64];
    let mut received = 0u32;
    let mut echoed = 0u32;
    let mut failed = false;
    while start.elapsed() < FuriDuration::from_secs(60) {
        // Board's Back input is active low; read without reconfiguring it.
        if !unsafe { sys::furi_hal_gpio_read(&sys::gpio_button_back) } {
            break;
        }
        match serial.read(&mut data, FuriDuration::from_millis(50)) {
            Ok(0) => {}
            Ok(count) => {
                received += count as u32;
                if serial
                    .tx(&data[..count], FuriDuration::from_millis(500))
                    .is_err()
                {
                    failed = true;
                    break; // Delivery unknown: do not retry or silently lose data.
                }
                echoed += count as u32;
            }
            Err(_) => {
                failed = true;
                break;
            }
        }
    }
    info!(
        "BLE echo received={} confirmed={} dropped={}",
        received,
        echoed,
        serial.dropped_bytes()
    );
    unsafe {
        sys::view_port_enabled_set(view, false);
        sys::gui_remove_view_port(gui.as_ptr(), view);
        sys::view_port_free(view);
    }
    drop(serial);
    if failed { 1 } else { 0 }
}
