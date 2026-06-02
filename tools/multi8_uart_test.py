#!/usr/bin/env python3
"""UART smoke test for multi8 firmware without SDL controllers.

Examples:
    python tools/multi8_uart_test.py --port COM3 --slots 2 --announce
    python tools/multi8_uart_test.py --port /dev/ttyUSB0 --slots 4 --colors
"""
from __future__ import annotations

import argparse
import pathlib
import sys
import time

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src"))

from switch_pico_bridge.switch_pico_uart import PicoUART, SwitchButton, SwitchDpad, SwitchReport  # noqa: E402

PRESETS = [
    ((0xFF, 0x2A, 0x2A), (0x20, 0x10, 0x10)),
    ((0x00, 0x80, 0xFF), (0x10, 0x18, 0x28)),
    ((0x00, 0xD0, 0x70), (0x10, 0x20, 0x16)),
    ((0xFF, 0xD0, 0x20), (0x30, 0x24, 0x08)),
    ((0x9B, 0x5C, 0xFF), (0x18, 0x10, 0x28)),
    ((0x00, 0xD5, 0xFF), (0x08, 0x24, 0x28)),
    ((0xFF, 0x4D, 0xB8), (0x28, 0x10, 0x20)),
    ((0xF4, 0xF4, 0xF4), (0x20, 0x20, 0x20)),
]


def make_report(buttons: int = 0) -> SwitchReport:
    return SwitchReport(buttons=buttons, hat=SwitchDpad.CENTER, lx=128, ly=128, rx=128, ry=128)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--port", required=True)
    parser.add_argument("--baud", type=int, default=921600)
    parser.add_argument("--slots", type=int, default=2)
    parser.add_argument("--seconds", type=float, default=20.0)
    parser.add_argument("--hz", type=float, default=250.0)
    parser.add_argument("--announce", action="store_true")
    parser.add_argument("--colors", action="store_true")
    args = parser.parse_args()

    slots = max(1, min(8, args.slots))
    uart = PicoUART(args.port, args.baud)
    try:
        if args.colors:
            for slot in range(slots):
                body, buttons = PRESETS[slot % len(PRESETS)]
                uart.set_color(slot, body, buttons)
                time.sleep(0.03)
        if args.announce:
            for slot in range(slots):
                uart.announce(slot, duration_ms=120)
                time.sleep(0.35)

        period = 1.0 / max(args.hz, 1.0)
        end = time.monotonic() + max(args.seconds, 0.1)
        neutral = make_report()
        buttons = [SwitchButton.A, SwitchButton.B, SwitchButton.X, SwitchButton.Y, SwitchButton.L, SwitchButton.R, SwitchButton.ZL, SwitchButton.ZR]
        tick = 0
        while time.monotonic() < end:
            for slot in range(slots):
                phase = (tick // int(max(args.hz // 2, 1))) % (slots + 1)
                report = make_report(int(buttons[slot % len(buttons)])) if phase == slot else neutral
                uart.send_report(report, controller_id=slot)
            tick += 1
            time.sleep(period)
    finally:
        for slot in range(slots):
            try:
                uart.send_report(make_report(), controller_id=slot)
            except Exception:
                pass
        uart.close()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
