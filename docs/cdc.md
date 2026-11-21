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
3. Within the 35-second app lifetime, send synthetic packets of 1, 7, 63,
   and 64 bytes, one at a time. Require an exact echo within a bounded host
   timeout before sending the next. Do not pipeline. Record mismatches/timeouts.
4. Send a reserved single-byte EOT (`0x04`) packet after the final echo; none of
   the synchronous test payloads may equal this command. Wait for exactly
   `CDC-ASYNC-256\n`. The app installs its worker and callbacks before this reply.
5. Send exactly 256 synthetic bytes. Async reception counts bytes, regardless
   of USB packet boundaries or stream-buffer chunking. Expect eight reply bytes:
   ASCII `CDC!` followed by the count as a little-endian `u32`. Require count 256.
   Send nothing else. The app allows 100 ms for that reply then restores USB.
6. At most 35 seconds after launch (plus 100 ms response grace), the app drops the
   receiver, joins its worker, and restores the original interface, even if the
   host never completes the protocol. Reopen the original CLI, verify it responds,
   and verify the app is no longer running. Repeat twice.
7. In a separately coordinated run, disconnect/reconnect the host during the
   receive window. Confirm the timer still terminates, the CLI recovers, and the
   next run works. Missing traffic during disconnect is expected.

`scripts/test-cdc-echo.py APPLICATION_PORT` automates the synthetic protocol using
pyserial with bounded read/write timeouts. It never discovers a port, launches an
app, or flashes firmware. Pass only the identified application port, and verify
CLI restoration separately. This tests byte counts; async payload integrity and
queue overflow are not established by a count-only response.

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

The worker checks STOP before every queued callback without clearing notification
flags. Momentum mntm-012 `furi/core/thread.c` implements `furi_thread_flags_get`
with FreeRTOS `eNoAction`, so checking STOP preserves a racing DATA notification
for the next outer wait. Drop already detaches the producer before joining;
this change improves response to cancellation within the bounded backlog rather
than fixing an established unbounded-producer hang. No simulated scheduling test
is claimed; interrupt and worker behavior still require on-device acceptance.
