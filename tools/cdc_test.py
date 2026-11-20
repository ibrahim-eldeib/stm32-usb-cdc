#!/usr/bin/env python3
# SPDX-License-Identifier: MIT — Copyright (c) 2026 ibrahim-eldeib
"""
Host-side check for a board running stm32-usb-cdc.

  python tools/cdc_test.py                 # auto-detect by VID:PID, show what the board prints
  python tools/cdc_test.py --echo          # loopback test (needs examples/bare_metal_echo.c firmware)
  python tools/cdc_test.py --port COM8 --vid 0x0483 --pid 0x5740

Requires: pip install pyserial
"""
import argparse
import os
import sys
import time

import serial
from serial.tools import list_ports


def find(vid, pid):
    return [p for p in list_ports.comports() if p.vid == vid and p.pid == pid]


def open_port(name, retries=5):
    # Opening right after another program closed the port can fail once on
    # Windows ("device not functioning"); retrying a second later works.
    for i in range(retries):
        try:
            return serial.Serial(name, 115200, timeout=0.5)
        except serial.SerialException as e:
            if i == retries - 1:
                raise
            print(f"open failed ({e}); retrying")
            time.sleep(1)


def monitor(s, seconds):
    t = time.time()
    while time.time() - t < seconds:
        data = s.read(s.in_waiting or 1)
        if data:
            sys.stdout.write(data.decode(errors="replace"))
            sys.stdout.flush()


def echo_test(s, total=64 * 1024):
    s.reset_input_buffer()
    payload = os.urandom(total)
    got = bytearray()
    t0 = time.time()
    chunk = 512
    for i in range(0, total, chunk):
        s.write(payload[i:i + chunk])
        got += s.read(s.in_waiting)
    deadline = time.time() + 5
    while len(got) < total and time.time() < deadline:
        got += s.read(max(1, s.in_waiting))
    dt = time.time() - t0
    ok = bytes(got[:total]) == payload
    print(f"echo: sent {total} B, got {len(got)} B, {'MATCH' if ok else 'MISMATCH'}, "
          f"{2 * total / dt / 1024:.0f} KiB/s both directions")
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port")
    ap.add_argument("--vid", type=lambda x: int(x, 0), default=0x0483)
    ap.add_argument("--pid", type=lambda x: int(x, 0), default=0x5740)
    ap.add_argument("--echo", action="store_true", help="run the loopback test")
    ap.add_argument("--seconds", type=float, default=5, help="monitor time")
    a = ap.parse_args()

    name = a.port
    if not name:
        ports = find(a.vid, a.pid)
        if not ports:
            print(f"no port with VID:PID {a.vid:04X}:{a.pid:04X}. Present ports:")
            for p in list_ports.comports():
                print(f"  {p.device}  {p.vid and hex(p.vid)}:{p.pid and hex(p.pid)}  {p.description}")
            sys.exit(1)
        p = ports[0]
        name = p.device
        print(f"found {name}  serial={p.serial_number}  '{p.description}'")

    s = open_port(name)
    s.dtr = True   # tells the firmware a terminal is open (USB_CDC_IsPortOpen)
    try:
        if a.echo:
            sys.exit(0 if echo_test(s) else 2)
        monitor(s, a.seconds)
    finally:
        s.close()


if __name__ == "__main__":
    main()
