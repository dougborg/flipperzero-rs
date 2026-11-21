#!/usr/bin/env python3
"""Bounded BLE echo acceptance for the running ble-serial-echo example.

Requires bleak and pyserial. Does not upload/start apps, alter settings, unpair,
inspect keys, or reconnect automatically. Reserve exclusive Flipper USB access.
Only byte counts are printed; peripheral addresses and raw captures are omitted.
"""
import argparse
import asyncio
import json
import time

import serial
from bleak import BleakClient, BleakScanner

TX = '19ed82ae-ed21-4c9d-4145-228e61fe0000'
RX = '19ed82ae-ed21-4c9d-4145-228e62fe0000'
FLOW = '19ed82ae-ed21-4c9d-4145-228e63fe0000'
GAP_NAME = '00002a00-0000-1000-8000-00805f9b34fb'
WINDOW = 2048


def loader_running(args):
    """Independent USB check. Never print the CLI banner/device identity."""
    def prompt(port):
        data = bytearray()
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            data.extend(port.read(max(1, port.in_waiting)))
            if data.endswith(b'>: '):
                return data.decode(errors='replace')
        raise TimeoutError('USB CLI prompt deadline')

    with serial.Serial(args.cli_port, 115200, timeout=.1, write_timeout=2) as port:
        port.dtr = True
        time.sleep(.5)
        port.reset_input_buffer()
        port.write(b'\r')
        prompt(port)
        port.write(b'loader info\r')
        reply = prompt(port)
    if f'Application "{args.app_name}" is running' not in reply:
        raise RuntimeError('Expected application is not running; refusing BLE writes')


async def session(args, result, length):
    result['phase'] = 'discovery'
    await asyncio.to_thread(loader_running, args)
    discovered = await BleakScanner.discover(timeout=3, return_adv=True)
    matches = [device for device, adv in discovered.values()
               if adv.local_name == args.name and
               (args.device is None or device.address == args.device)]
    if len(matches) != 1:
        raise RuntimeError('Expected exactly one fresh application advertisement')
    disconnected = False

    def on_disconnect(_):
        nonlocal disconnected
        disconnected = True  # Sticky: never reuse a connection after profile restore.

    stats = {'requested': length, 'sent': 0, 'received': 0, 'grants': []}
    result['sessions'].append(stats)
    result['phase'] = 'connect'
    async with BleakClient(matches[0], timeout=10, disconnected_callback=on_disconnect) as client:
        if client.name != args.name:
            raise RuntimeError('Connected peripheral name does not identify the app')
        if client.services.get_characteristic(GAP_NAME):
            if bytes(await client.read_gatt_char(GAP_NAME)).decode() != args.name:
                raise RuntimeError('GATT device name does not identify the app')
            stats['gatt_name_read'] = True
        else:
            # CoreBluetooth may hide Generic Access. Keep all independent gates:
            # fresh advertisement, current peripheral name, characteristics, USB app.
            stats['gatt_name_unavailable'] = True
        for uuid, prop in ((TX, 'indicate'), (RX, 'write'), (FLOW, 'notify')):
            characteristic = client.services.get_characteristic(uuid)
            if not characteristic or prop not in characteristic.properties:
                raise RuntimeError('Required serial characteristic missing')

        received = bytearray()
        changed, granted = asyncio.Event(), asyncio.Event()
        credits = 0

        def on_rx(_, data):
            received.extend(data)
            stats['received'] += len(data)
            changed.set()

        def on_credit(_, data):
            nonlocal credits
            if len(data) != 4:
                return  # Malformed grants cannot authorize more writes.
            value = int.from_bytes(data, 'big')
            if value != WINDOW:
                return
            credits = value  # Firmware grants a window, not an increment.
            stats['grants'].append(value)
            granted.set()

        await client.start_notify(TX, on_rx)
        await client.start_notify(FLOW, on_credit)
        epoch = len(stats['grants'])
        initial = int.from_bytes(await client.read_gatt_char(FLOW), 'big')
        if len(stats['grants']) == epoch:
            credits = initial  # Do not overwrite a newer notification with stale read.
        if credits != WINDOW:
            raise RuntimeError('Expected a fresh/replenished window boundary')
        await asyncio.to_thread(loader_running, args)  # Immediately before writes.
        payload = bytes((i * 17 + 0x41) & 255 for i in range(length))
        offset = 0
        while offset < length:
            while credits == 0:
                result['phase'] = 'wait_credit'
                granted.clear()
                await asyncio.wait_for(granted.wait(), 12)
            if disconnected or not client.is_connected:
                raise RuntimeError('Disconnected; refusing further writes')
            # 2048 is NOT divisible by 20. Consume the final 8 bytes; waiting for
            # another 20-byte grant would deadlock against firmware's zero threshold.
            chunk = payload[offset:offset + min(20, credits, length - offset)]
            credits -= len(chunk)
            result['phase'] = 'write'
            await client.write_gatt_char(RX, chunk, response=True)
            offset += len(chunk)
            stats['sent'] += len(chunk)
        result['phase'] = 'wait_echo'
        while len(received) < length:
            changed.clear()
            await asyncio.wait_for(changed.wait(), 12)
        if bytes(received) != payload:
            raise RuntimeError('Echo mismatch or trailing bytes')
        stats['exact_echo'] = True
        # Receiving an indication does not mean the firmware has processed its ACK.
        result['phase'] = 'settle_confirmation'
        await asyncio.sleep(.5)
        while credits != WINDOW:
            granted.clear()
            await asyncio.wait_for(granted.wait(), 3)
        await asyncio.to_thread(loader_running, args)
        stats['credits_at_disconnect'] = credits
        stats['loader_live_before_disconnect'] = True


async def run(args, result):
    # Reconnect only at a fully replenished window. An arbitrary reconnect cannot
    # treat the characteristic's LAST grant as its remaining balance.
    await session(args, result, WINDOW)
    await asyncio.sleep(.5)
    await session(args, result, WINDOW * 2)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--cli-port', required=True, help='Flipper USB CLI port; exclusive access required')
    parser.add_argument('--device', help='Optional BLE address/UUID to disambiguate multiple test peripherals')
    parser.add_argument('--name', default='BLE Serial App', help='Distinct test-app advertised/peripheral name')
    parser.add_argument('--app-name', default='BLE Serial Echo', help='Expected USB loader application name')
    args = parser.parse_args()
    result = {'sessions': [], 'hardware_dropped_counter': 'not observed'}
    try:
        asyncio.run(asyncio.wait_for(run(args, result), 50))
        result['passed'] = True
    except Exception as error:
        result['passed'] = False
        # Exception strings from BLE libraries may contain private device IDs.
        result['error_type'] = type(error).__name__
    print(json.dumps(result, indent=2))
    raise SystemExit(0 if result['passed'] else 1)


if __name__ == '__main__':
    main()
