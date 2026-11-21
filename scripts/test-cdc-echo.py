#!/usr/bin/env python3
"""Bounded synthetic CDC test; pass the identified *application* CDC port.

Requires pyserial. Never point this at the firmware CLI port. Launch cdc-echo
first, then run within its 35-second lifetime. No device discovery or flashing.
"""
import argparse
import struct
import time

import serial


def read_exact(port, length):
    deadline = time.monotonic() + 3
    received = bytearray()
    while len(received) < length and time.monotonic() < deadline:
        received.extend(port.read(length - len(received)))
    if len(received) != length:
        raise RuntimeError(f"response timeout: expected {length}, got {len(received)} bytes")
    return bytes(received)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("application_port")
    args = parser.parse_args()
    with serial.Serial(args.application_port, 115200, timeout=0.25, write_timeout=3) as port:
        port.dtr = True
        for length in (1, 7, 63, 64):
            payload = bytes((index * 17 + 0x41) & 255 for index in range(length))
            port.write(payload)
            if read_exact(port, length) != payload:
                raise RuntimeError(f"{length}-byte echo mismatch")
            print(f"{length}-byte echo passed")
        port.write(b"\x04")  # Reserved one-byte phase transition, not a test payload.
        ready = b"CDC-ASYNC-256\n"
        if read_exact(port, len(ready)) != ready:
            raise RuntimeError("async readiness mismatch")
        # Two 128-byte host writes exercise splitting into 64-byte USB packets.
        port.write(bytes(range(128)))
        port.write(bytes(range(128, 256)))
        reply = read_exact(port, 8)
        if reply[:4] != b"CDC!" or struct.unpack("<I", reply[4:])[0] != 256:
            raise RuntimeError(f"async count mismatch: {reply!r}")
        print("async count passed: 256 bytes")
    print("Device should restore its original interface; verify CLI separately.")


if __name__ == "__main__":
    main()
