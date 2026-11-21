# USB CDC integration and acceptance

This wrapper is experimental. Compilation is not hardware acceptance.
`UsbCdc::enable` is unsafe because the firmware exposes global USB state rather
than an atomic channel lease. Its safety requirements apply until teardown.

## Firmware ownership constraints

- Prefer dual CDC: firmware CLI remains on channel 0 and the app uses channel 1.
- Single CDC requires explicitly stopping and later restoring the firmware CLI.
  Merely selecting `usb_cdc_single` does not release its callback slot. This
  example does not attempt a firmware-specific CLI handoff.
- Stop competing USB apps, RPC clients, and mode changes before acquisition.
  HAL check/switch/lock calls do not constitute an atomic ownership claim.
- HAL returns the previous interface pointer but not its context. Restore only
  interfaces known to support a null initialization context. The diagnostic
  refuses to start unless the original interface is `usb_cdc_single`.
- Callback registration is synchronized with USB interrupts/task switches using
  Furi critical sections. Only this wrapper's nonblocking callbacks (or null)
  may occupy the selected channel. Do not substitute external callbacks.
- Async reception consumes the handle. Its callback must own its captures,
  implement `Send`, and return promptly. Forgetting it leaks the worker and USB
  lease; never unload a FAP while a leaked worker remains active.
- The 2048-byte receive queue can overflow. TX submits at most one 64-byte packet
  without completion or acceptance feedback; disconnected/busy writes may be
  lost. This is not yet a reliable serial stream API or a throughput benchmark.

## Bounded diagnostic

Build `cargo build -p flipperzero --example cdc-echo --release` from `crates/`.
Before installation, audit the FAP's API version and imports against the exact
installed firmware SDK. SDK 0.16 targets API 87.1 (including Momentum mntm-012);
current upstream SDK 0.17 targets a different API. Do not upgrade firmware just
to run the example. Use a dedicated test filename and leave existing apps intact.

1. Obtain exclusive device access. Close qFlipper, RPC clients, serial monitors,
   and other tests. Record the original interface and verify CLI responsiveness.
2. Launch the diagnostic. USB re-enumerates to two CDC ports. Identify the app
   port by interface number, not a hard-coded device path; do not send arbitrary
   binary data to the CLI port. Assert DTR on the application port.
3. For the first 30 seconds, send synthetic packets of 1, 7, 63, and 64 bytes,
   one at a time. Require an exact echo within a bounded host timeout before
   sending the next. Do not pipeline. Record mismatches and timeouts.
4. The next five seconds receive asynchronously without echo. Send a known
   bounded byte count, then stop. After restoration compare the reported async
   count with what was sent. Overflow testing is a separate lossy stress test.
5. At 35 seconds the app drops the receiver, joins its worker, and restores the
   original interface. Allow re-enumeration, reopen the original CLI, verify it
   responds, and verify the app is no longer running. Repeat twice.
6. In a separately coordinated run, disconnect/reconnect the host during the
   receive window. Confirm the timer still terminates, the CLI recovers, and the
   next run works. Missing traffic during disconnect is expected.

Do not claim single-port, sustained-throughput, overflow accounting, or arbitrary
previous-mode restoration from this diagnostic. Physical reconnect and CLI
handoff tests require an available operator and an agreed recovery procedure.

## Source audit

The ownership and callback behavior was checked against
[Momentum mntm-012 CDC HAL](https://github.com/Next-Flip/Momentum-Firmware/blob/mntm-012/targets/f7/furi_hal/furi_hal_usb_cdc.c),
[its CLI service](https://github.com/Next-Flip/Momentum-Firmware/blob/mntm-012/applications/services/cli/cli_vcp.c),
and the corresponding official firmware sources. Callback slots survive CDC
interface reinitialization; mode selection alone does not transfer ownership.
No on-device acceptance results are recorded yet.

## Software checks

`python3 scripts/check-cdc-contracts.py` verifies the compiler rejects borrowed
captures, non-`Send` captures, and reuse of the consumed USB owner. It does not
simulate firmware scheduling or prove interrupt safety. Build the positive
`cdc-echo` example as well so a general build failure cannot masquerade as a
successful negative test.

The revised wrapper and example were checked with SDK 0.16 and, in a separate
checkout, against upstream 0.17. The SDK 0.16 release FAP passed static manifest
and import checks against Momentum mntm-012 API 87.1. No firmware was installed
or upgraded. On Rust 1.98, strict Clippy encounters existing generated runtime
symbol signatures and `manual_checked_ops` in `furi/time.rs`; these are separate
from CDC and must not be described as an unqualified clean upstream lint run.
