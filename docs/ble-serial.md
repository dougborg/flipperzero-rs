# Application BLE serial (experimental)

`bluetooth::serial::BleSerial` is a peripheral serial service, not a BLE central
or a client for the Flipper RPC service. It uses a distinct profile template and
advertises `BLE Serial App`. Firmware RPC only registers its callbacks when the
active template is `ble_profile_serial`, so it does not own this callback slot.
The GATT serial UUIDs and credit/indication protocol remain Flipper-compatible.
Device-info, battery and serial services retain the default allocation order to
avoid changing cached handles for the existing peripheral identity. Runtime
cache/rediscovery behavior must still be validated on each central platform.

This revision replaces the original borrowed asynchronous receiver with an owned,
polled session. `read(&mut self, ...)` has one reader; the BLE event thread is the
one writer. No user closure runs in the firmware callback or a Rust worker thread.
The session is neither Send nor Sync. Constructor safety requires the caller to
exclusively own the global profile and drop it before application code unloads.
The firmware record API cannot enforce that global lease. Never run Bluetooth
test modes or another profile-changing API while holding this session.

Receive storage is 4,096 bytes, with 2,048 advertised credits. One extra window
absorbs a receive racing between the empty observation and credit replenishment. Firmware credits are replenished when the queue
is drained. Peers must honor credits; `dropped_bytes()` reports rejected bytes if
they do not. Callback return is remaining capacity. `tx` limits packets to the
default ATT payload of 20 bytes and waits for an indication confirmation. The
characteristic's 486-byte storage limit does not establish negotiated MTU; larger
application writes must be split into <=20-byte calls. The echo example drains
20-byte chunks even when the central writes a larger RX payload. A timeout
means delivery is unknown, not failed; pending confirmation is retained and a
second indication is not submitted until it arrives. A failed low-level TX is
also conservatively left pending. Reload the session after an unrecoverable
transport failure; do not retry an unknown-delivery packet blindly. Peer reset
is sticky and requires dropping/recreating the session.

Credits belong to the running profile, not an individual central connection.
The flow characteristic holds the last full grant, not the remaining balance.
A reconnecting client must preserve its outstanding credit balance, or reconnect
at a known fully replenished boundary; rereading the characteristic after an
arbitrary disconnect is not a fresh grant. Drop/recreate the profile when a fresh
session is required. Reconnect with an outstanding unconfirmed indication is not
claimed: the example exits on confirmation timeout instead of resending unknown
bytes. Wait for the final indication acknowledgement before an idle reconnect.

## Lifecycle reviewed against Momentum mntm-012

Exact firmware commit: `e1784e7418d8b074e971983ceb6fef0f37e52ae4`, API87.1.

1. `bt_profile_start` synchronously sends a profile-change request to the BT
   service. `bt_change_profile` only starts advertising after `start` returns.
   The start hook allocates the serial service and installs its callback/context
   before returning. There is no live callback replacement. Service-allocation
   failure is fail-stop: the firmware serial-service allocator can leave a dangling
   dispatcher entry on failure, so normal recovery is not sound.
2. Drop calls `bt_profile_restore_default` synchronously, keeping Context alive.
   The firmware's `furi_hal_bt_change_app` calls `furi_hal_bt_reinit`, which stops
   advertising/disconnects, calls our stop hook to unregister/destroy the serial
   service, and subsequently calls `ble_glue_stop`.
3. `ble_glue_stop` calls `ble_event_thread_stop`, which joins the BLE event worker.
   The callback context is freed only after the synchronous restore returns.
   This relies on the firmware profile lifecycle, not on the unsynchronized
   callback setter as a teardown barrier. Restoration failure is fail-stop rather
   than freeing potentially referenced storage.
4. The custom stop hook does not free Context. Firmware-owned service allocation
   and event-dispatcher removal follow the same lifecycle as built-in profiles.

Relevant firmware sources: `applications/services/bt/bt_service/bt_api.c`,
`bt.c`; `targets/f7/furi_hal/furi_hal_bt.c`; and
`targets/f7/ble_glue/{ble_glue.c,ble_event_thread.c,services/serial_service.c}`.
This source review is not a substitute for runtime load/exit/reconnect testing.

## Build and validation

From `crates`, build `cargo build -p flipperzero --example ble-serial-echo --features alloc`.
Run `python3 scripts/check-ble-contracts.py` from the repository root with Cargo
and the ARM toolchain on PATH. It verifies that safe construction, moving/sharing
across threads, and immutable I/O are rejected by the compiler.
`python3 scripts/check-ble-credit-window.py` exhaustively models small credit
windows and all read/receive/notify interleavings, reproducing the one-window
overflow and verifying the two-window bound for credit-honoring peers.

The API87.1 artifact must be built with SDK0.16.0, not the current SDK0.17.0/API88.2.
Never modify only the manifest to bypass firmware API compatibility checks.
No hardware validation is claimed yet.

## Bounded hardware acceptance

The example displays a screen, checks Back at least every receive/TX timeout,
exits after 60 seconds plus at most its bounded final TX wait, and reports only
byte counters. It exits immediately on uncertain TX or peer reset. It leaves
Bluetooth settings and saved pairing keys unchanged through application code;
normal firmware pairing behavior still applies.

1. Coordinate exclusive USB, audit the artifact against the exact installed SDK,
   load with no central, test Back, and verify default profile/CLI restoration.
2. Reload. Confirm both advertised name and GATT device name identify
   `BLE Serial App` before sending any data. The default profile is RPC; arbitrary
   test bytes must not be sent after the example exits/restores it.
3. Connect using existing pairing, or ask the user to confirm pairing if needed.
   Subscribe to TX indications and flow-credit notifications. Honor credits.
   Check deterministic 1-, 20- and 64-byte payloads and a bounded 4KiB stream.
4. Disconnect/reconnect while idle and repeat. Verify matching bytes and zero
   rejected bytes; exercise peer reset separately.
5. Test exit with traffic pending, then two load/exit cycles, checking default
   profile and CLI responsiveness each time. Stop on any fault. Do not replace
   firmware, inspect credentials, remove pairings, or reset settings.
