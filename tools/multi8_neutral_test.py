#!/usr/bin/env python3
"""Neutral 8-slot UART test: colors and announce only, no simulated button presses."""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from switch_pico_bridge.switch_pico_uart import PicoUART, SwitchDpad, SwitchReport  # noqa: E402

PRESETS = [
    ((0x00, 0x00, 0xFF), (0x00, 0x00, 0xFF)),
    ((0xFF, 0x00, 0x00), (0xFF, 0x00, 0x00)),
    ((0x00, 0xFF, 0x00), (0x00, 0xFF, 0x00)),
    ((0xFF, 0xFF, 0x00), (0xFF, 0xFF, 0x00)),
    ((0x80, 0x00, 0xFF), (0x80, 0x00, 0xFF)),
    ((0xFF, 0x80, 0x00), (0xFF, 0x80, 0x00)),
    ((0x00, 0xFF, 0xFF), (0x00, 0xFF, 0xFF)),
    ((0xFF, 0x00, 0xFF), (0xFF, 0x00, 0xFF)),
]


def neutral_report() -> SwitchReport:
    return SwitchReport(buttons=0, hat=SwitchDpad.CENTER, lx=128, ly=128, rx=128, ry=128)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--slots", type=int, default=8)
    parser.add_argument("--seconds", type=float, default=300.0)
    parser.add_argument("--hz", type=float, default=250.0)
    parser.add_argument("--announce", action="store_true")
    args = parser.parse_args()

    slots = max(1, min(8, args.slots))
    report = neutral_report()
    uart = PicoUART(args.port, args.baud)
    try:
        for slot in range(slots):
            body, buttons = PRESETS[slot]
            uart.set_color(slot, body, buttons)
            print(f"slot {slot}: color body={body} buttons={buttons}", flush=True)
            time.sleep(0.03)
        if args.announce:
            for slot in range(slots):
                uart.announce(slot, duration_ms=120)
                print(f"slot {slot}: announce", flush=True)
                time.sleep(0.35)

        period = 1.0 / max(args.hz, 1.0)
        end = time.monotonic() + max(args.seconds, 0.1)
        while time.monotonic() < end:
            for slot in range(slots):
                uart.send_report(report, controller_id=slot)
            time.sleep(period)
    finally:
        for slot in range(slots):
            try:
                uart.send_report(report, controller_id=slot)
            except Exception:
                pass
        uart.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
