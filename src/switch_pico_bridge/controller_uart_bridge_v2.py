#!/usr/bin/env python3
"""V2 explicit multi-slot CLI wrapper for switch-pico.

This module is intentionally a wrapper:
- if --port is not present, it delegates to the original controller_uart_bridge.main();
- if --port is present, it runs the new single-Pico multi-slot mode:

    controller-uart-bridge --port COM3 \
      --map 0:0 --map 1:1 \
      --color 0:red --color 1:blue \
      --announce
"""
from __future__ import annotations

import argparse
import ctypes
import sys
import time
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple

import sdl3
from rich.console import Console
from rich.table import Table
from serial import SerialException

from . import controller_uart_bridge as legacy
from .switch_pico_uart import (
    UART_BAUD,
    PicoUART,
    SwitchButton,
    SwitchDpad,
    SwitchReport,
    axis_to_stick,
    str_to_dpad,
    trigger_to_button,
    decode_rumble,
)

MAX_SWITCH_SLOTS = 8

PRESET_COLORS: Dict[str, Tuple[Tuple[int, int, int], Tuple[int, int, int]]] = {
    "blue": ((0x00, 0x80, 0xFF), (0x00, 0x80, 0xFF)),
    "red": ((0xFF, 0x2A, 0x2A), (0xFF, 0x2A, 0x2A)),
    "green": ((0x00, 0xD0, 0x70), (0x00, 0xD0, 0x70)),
    "yellow": ((0xFF, 0xD0, 0x20), (0xFF, 0xD0, 0x20)),
    "purple": ((0x9B, 0x5C, 0xFF), (0x9B, 0x5C, 0xFF)),
    "orange": ((0xFF, 0x80, 0x20), (0xFF, 0x80, 0x20)),
    "cyan": ((0x00, 0xD5, 0xFF), (0x00, 0xD5, 0xFF)),
    "black": ((0x20, 0x20, 0x20), (0x20, 0x20, 0x20)),
    "pink": ((0xFF, 0x4D, 0xB8), (0xFF, 0x4D, 0xB8)),
    "white": ((0xF4, 0xF4, 0xF4), (0xF4, 0xF4, 0xF4)),
    "gray": ((0x80, 0x80, 0x80), (0x80, 0x80, 0x80)),
    "p0": ((0x00, 0x80, 0xFF), (0x00, 0x80, 0xFF)),
    "p1": ((0xFF, 0x2A, 0x2A), (0xFF, 0x2A, 0x2A)),
    "p2": ((0x00, 0xD0, 0x70), (0x00, 0xD0, 0x70)),
    "p3": ((0xFF, 0xD0, 0x20), (0xFF, 0xD0, 0x20)),
    "p4": ((0x9B, 0x5C, 0xFF), (0x9B, 0x5C, 0xFF)),
    "p5": ((0xFF, 0x80, 0x20), (0xFF, 0x80, 0x20)),
    "p6": ((0x00, 0xD5, 0xFF), (0x00, 0xD5, 0xFF)),
    "p7": ((0x20, 0x20, 0x20), (0x20, 0x20, 0x20)),
}

BUTTON_MAP_DEFAULT = legacy.BUTTON_MAP
DPAD_BUTTONS = legacy.DPAD_BUTTONS
STICK_AXES = legacy.STICK_AXES
RUMBLE_MIN_ACTIVE = getattr(legacy, "RUMBLE_MIN_ACTIVE", 0.40)
RUMBLE_IDLE_TIMEOUT = getattr(legacy, "RUMBLE_IDLE_TIMEOUT", 0.25)


def parse_map(value: str) -> Tuple[int, int]:
    if ":" not in value:
        raise argparse.ArgumentTypeError("mapping must look like PC:SWITCH, for example 0:0")
    left, right = value.split(":", 1)
    try:
        pc = int(left, 10)
        slot = int(right, 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("mapping must use integer ids, for example 0:0") from exc
    if pc < 0:
        raise argparse.ArgumentTypeError("PC controller id must be >= 0")
    if slot < 0 or slot >= MAX_SWITCH_SLOTS:
        raise argparse.ArgumentTypeError(f"Switch slot id must be between 0 and {MAX_SWITCH_SLOTS - 1}")
    return pc, slot


def _parse_hex_rgb(value: str) -> Tuple[int, int, int]:
    v = value.strip()
    if v.startswith("#"):
        v = v[1:]
    if len(v) != 6:
        raise argparse.ArgumentTypeError(f"invalid RGB color '{value}', expected #RRGGBB")
    try:
        return int(v[0:2], 16), int(v[2:4], 16), int(v[4:6], 16)
    except ValueError as exc:
        raise argparse.ArgumentTypeError(f"invalid RGB color '{value}', expected #RRGGBB") from exc


def parse_color(value: str) -> Tuple[int, Tuple[int, int, int], Tuple[int, int, int], str]:
    parts = value.split(":")
    if len(parts) < 2:
        raise argparse.ArgumentTypeError("color must look like SWITCH:preset or SWITCH:#RRGGBB:#RRGGBB")
    try:
        slot = int(parts[0], 10)
    except ValueError as exc:
        raise argparse.ArgumentTypeError("color slot must be an integer") from exc
    if slot < 0 or slot >= MAX_SWITCH_SLOTS:
        raise argparse.ArgumentTypeError(f"color slot must be between 0 and {MAX_SWITCH_SLOTS - 1}")
    if len(parts) == 2:
        preset = parts[1].strip().lower()
        if preset not in PRESET_COLORS:
            known = ", ".join(sorted(PRESET_COLORS))
            raise argparse.ArgumentTypeError(f"unknown color preset '{preset}'. Known: {known}")
        body, buttons = PRESET_COLORS[preset]
        return slot, body, buttons, preset
    if len(parts) == 3:
        body = _parse_hex_rgb(parts[1])
        buttons = _parse_hex_rgb(parts[2])
        return slot, body, buttons, f"#{body[0]:02X}{body[1]:02X}{body[2]:02X}/#{buttons[0]:02X}{buttons[1]:02X}{buttons[2]:02X}"
    raise argparse.ArgumentTypeError("color must look like SWITCH:preset or SWITCH:#RRGGBB:#RRGGBB")


@dataclass
class MultiContext:
    controller: sdl3.SDL_Gamepad
    pc_index: int
    switch_slot: int
    instance_id: int
    name: str
    report: SwitchReport = field(default_factory=SwitchReport)
    dpad: Dict[str, bool] = field(default_factory=lambda: {"up": False, "down": False, "left": False, "right": False})
    button_state: Dict[int, bool] = field(default_factory=dict)
    last_trigger_state: Dict[str, bool] = field(default_factory=lambda: {"left": False, "right": False})
    last_send: float = 0.0
    rumble_active: bool = False
    last_rumble: float = 0.0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="switch-pico explicit multi-slot bridge")
    parser.add_argument("--port", required=True, help="Pico UART serial port, e.g. COM3 or /dev/ttyUSB0")
    parser.add_argument("--map", action="append", type=parse_map, default=[], help="Map PC controller to Switch slot: PC:SWITCH, e.g. 0:0. Repeatable.")
    parser.add_argument("--color", action="append", type=parse_color, default=[], help="Set slot color: SWITCH:preset or SWITCH:#RRGGBB:#RRGGBB. Repeatable.")
    parser.add_argument("--announce", action="store_true", help="Pulse L+R on mapped slots after opening the bridge.")
    parser.add_argument("--announce-duration", type=int, default=120, help="Announce pulse duration in ms (default 120).")
    parser.add_argument("--announce-gap", type=float, default=0.35, help="Delay between slot announces in seconds (default 0.35).")
    parser.add_argument("--frequency", type=float, default=500.0, help="Report send frequency per controller (Hz, default 500).")
    parser.add_argument("--deadzone", type=float, default=0.08, help="Stick deadzone 0.0-1.0 (default 0.08).")
    parser.add_argument("--trigger-threshold", type=float, default=0.35, help="Trigger threshold 0.0-1.0 (default 0.35).")
    parser.add_argument("--baud", type=int, default=UART_BAUD, help=f"UART baud rate (default {UART_BAUD}).")
    parser.add_argument("--list-controllers", action="store_true", help="List detected SDL gamepads and exit.")
    parser.add_argument("--swap-abxy", action="store_true", help="Swap AB/XY for all mapped controllers.")
    return parser


def enumerate_gamepads() -> List[Tuple[int, int, str]]:
    count = ctypes.c_int(0)
    joystick_ids = sdl3.SDL_GetJoysticks(ctypes.byref(count))
    pads: List[Tuple[int, int, str]] = []
    if not joystick_ids:
        return pads
    try:
        display_idx = 0
        for i in range(count.value):
            instance_id = joystick_ids[i]
            if not sdl3.SDL_IsGamepad(instance_id):
                continue
            name = sdl3.SDL_GetGamepadNameForID(instance_id)
            name_str = name.decode(errors="ignore") if isinstance(name, bytes) else str(name) if name else "Unknown"
            pads.append((display_idx, instance_id, name_str))
            display_idx += 1
    finally:
        sdl3.SDL_free(joystick_ids)
    return pads


def list_controllers(console: Console) -> None:
    pads = enumerate_gamepads()
    if not pads:
        console.print("[yellow]No SDL gamepads detected.[/yellow]")
        return
    table = Table(title="Detected PC Controllers")
    table.add_column("PC id", justify="center")
    table.add_column("SDL instance", justify="center")
    table.add_column("Name")
    for display_idx, instance_id, name in pads:
        table.add_row(str(display_idx), str(instance_id), name)
    console.print(table)


def validate_maps(parser: argparse.ArgumentParser, maps: List[Tuple[int, int]]) -> None:
    pc_seen: Dict[int, int] = {}
    slot_seen: Dict[int, int] = {}
    for pc, slot in maps:
        if pc in pc_seen:
            parser.error(f"PC controller {pc} is mapped more than once")
        if slot in slot_seen:
            parser.error(f"Switch slot {slot} is mapped more than once")
        pc_seen[pc] = slot
        slot_seen[slot] = pc


def open_contexts(args: argparse.Namespace, console: Console) -> Dict[int, MultiContext]:
    pads = enumerate_gamepads()
    by_display = {display_idx: (instance_id, name) for display_idx, instance_id, name in pads}
    contexts: Dict[int, MultiContext] = {}
    for pc_index, switch_slot in args.map:
        if pc_index not in by_display:
            available = ", ".join(str(x[0]) for x in pads) or "none"
            raise RuntimeError(f"PC controller {pc_index} not found. Available PC ids: {available}")
        sdl_instance_id, name = by_display[pc_index]
        controller = sdl3.SDL_OpenGamepad(sdl_instance_id)
        if not controller:
            err = sdl3.SDL_GetError()
            err_str = err.decode(errors="ignore") if isinstance(err, bytes) else str(err)
            raise RuntimeError(f"Failed to open PC controller {pc_index}: {err_str}")
        joystick = sdl3.SDL_GetGamepadJoystick(controller)
        real_instance_id = sdl3.SDL_GetJoystickID(joystick)
        contexts[real_instance_id] = MultiContext(
            controller=controller,
            pc_index=pc_index,
            switch_slot=switch_slot,
            instance_id=real_instance_id,
            name=name,
        )
        console.print(f"[green]PC controller {pc_index} ({name}) -> Switch slot {switch_slot}[/green]")
    return contexts


def current_button_map(args: argparse.Namespace) -> Dict[int, SwitchButton]:
    if not args.swap_abxy:
        return dict(BUTTON_MAP_DEFAULT)
    swapped = dict(BUTTON_MAP_DEFAULT)
    swapped[sdl3.SDL_GAMEPAD_BUTTON_SOUTH] = SwitchButton.A
    swapped[sdl3.SDL_GAMEPAD_BUTTON_EAST] = SwitchButton.B
    swapped[sdl3.SDL_GAMEPAD_BUTTON_WEST] = SwitchButton.X
    swapped[sdl3.SDL_GAMEPAD_BUTTON_NORTH] = SwitchButton.Y
    return swapped


def poll_controller_buttons(ctx: MultiContext, button_map: Dict[int, SwitchButton]) -> None:
    for sdl_button, switch_bit in button_map.items():
        pressed = bool(sdl3.SDL_GetGamepadButton(ctx.controller, sdl_button))
        previous = ctx.button_state.get(sdl_button)
        if previous == pressed:
            continue
        ctx.button_state[sdl_button] = pressed
        if pressed:
            ctx.report.buttons |= switch_bit
        else:
            ctx.report.buttons &= ~switch_bit
    dpad_changed = False
    for sdl_button, name in DPAD_BUTTONS.items():
        pressed = bool(sdl3.SDL_GetGamepadButton(ctx.controller, sdl_button))
        if ctx.dpad[name] == pressed:
            continue
        ctx.dpad[name] = pressed
        dpad_changed = True
    if dpad_changed:
        ctx.report.hat = str_to_dpad(ctx.dpad)


def handle_axis_motion(event: sdl3.SDL_Event, contexts: Dict[int, MultiContext], deadzone_raw: int, trigger_threshold: int) -> None:
    ctx = contexts.get(event.gaxis.which)
    if not ctx:
        return
    axis = event.gaxis.axis
    value = int(event.gaxis.value)
    if axis == sdl3.SDL_GAMEPAD_AXIS_LEFTX:
        ctx.report.lx = axis_to_stick(value, deadzone_raw)
    elif axis == sdl3.SDL_GAMEPAD_AXIS_LEFTY:
        ctx.report.ly = axis_to_stick(value, deadzone_raw)
    elif axis == sdl3.SDL_GAMEPAD_AXIS_RIGHTX:
        ctx.report.rx = axis_to_stick(value, deadzone_raw)
    elif axis == sdl3.SDL_GAMEPAD_AXIS_RIGHTY:
        ctx.report.ry = axis_to_stick(value, deadzone_raw)
    elif axis == sdl3.SDL_GAMEPAD_AXIS_LEFT_TRIGGER:
        pressed = trigger_to_button(value, trigger_threshold)
        if pressed != ctx.last_trigger_state["left"]:
            if pressed:
                ctx.report.buttons |= SwitchButton.ZL
            else:
                ctx.report.buttons &= ~SwitchButton.ZL
            ctx.last_trigger_state["left"] = pressed
    elif axis == sdl3.SDL_GAMEPAD_AXIS_RIGHT_TRIGGER:
        pressed = trigger_to_button(value, trigger_threshold)
        if pressed != ctx.last_trigger_state["right"]:
            if pressed:
                ctx.report.buttons |= SwitchButton.ZR
            else:
                ctx.report.buttons &= ~SwitchButton.ZR
            ctx.last_trigger_state["right"] = pressed


def handle_button_event(event: sdl3.SDL_Event, contexts: Dict[int, MultiContext], button_map: Dict[int, SwitchButton]) -> None:
    ctx = contexts.get(event.gbutton.which)
    if not ctx:
        return
    button = event.gbutton.button
    pressed = event.type == sdl3.SDL_EVENT_GAMEPAD_BUTTON_DOWN
    if button in button_map:
        bit = button_map[button]
        if pressed:
            ctx.report.buttons |= bit
        else:
            ctx.report.buttons &= ~bit
        ctx.button_state[button] = pressed
    elif button in DPAD_BUTTONS:
        ctx.dpad[DPAD_BUTTONS[button]] = pressed
        ctx.report.hat = str_to_dpad(ctx.dpad)


def apply_colors(uart: PicoUART, args: argparse.Namespace, mapped_slots: set[int], console: Console) -> None:
    for slot, body, buttons, label in args.color:
        if slot not in mapped_slots:
            console.print(f"[yellow]Warning: color was provided for slot {slot}, but slot {slot} is not mapped.[/yellow]")
        uart.set_color(slot, body, buttons)
        console.print(f"[cyan]Color slot {slot}: {label}[/cyan]")


def announce_slots(uart: PicoUART, slots: List[int], args: argparse.Namespace, console: Console) -> None:
    for slot in sorted(slots):
        console.print(f"[cyan]Announce slot {slot} (L+R pulse)[/cyan]")
        uart.announce(slot, duration_ms=args.announce_duration)
        time.sleep(max(0.0, args.announce_gap))


def run_multi(args: argparse.Namespace) -> None:
    console = Console()
    validate_maps(build_parser(), args.map)
    if not sdl3.SDL_Init(sdl3.SDL_INIT_GAMEPAD | sdl3.SDL_INIT_JOYSTICK):
        err = sdl3.SDL_GetError()
        err_str = err.decode(errors="ignore") if isinstance(err, bytes) else str(err)
        raise RuntimeError(f"SDL init failed: {err_str}")
    contexts: Dict[int, MultiContext] = {}
    uart: Optional[PicoUART] = None
    try:
        if args.list_controllers:
            list_controllers(console)
            return
        contexts = open_contexts(args, console)
        mapped_slots = {slot for _, slot in args.map}
        uart = PicoUART(args.port, args.baud)
        console.print(f"[green]Opened Pico UART {args.port} at {args.baud} baud[/green]")
        apply_colors(uart, args, mapped_slots, console)
        if args.announce:
            announce_slots(uart, sorted(mapped_slots), args, console)
        button_map = current_button_map(args)
        interval = 1.0 / max(args.frequency, 1.0)
        deadzone_raw = int(max(0.0, min(args.deadzone, 1.0)) * 32767)
        trigger_threshold = int(max(0.0, min(args.trigger_threshold, 1.0)) * 32767)
        event = sdl3.SDL_Event()
        console.print("[green]Bridge running. Ctrl+C to stop.[/green]")
        running = True
        while running:
            while sdl3.SDL_PollEvent(ctypes.byref(event)):
                if event.type == sdl3.SDL_EVENT_QUIT:
                    running = False
                    break
                if event.type == sdl3.SDL_EVENT_GAMEPAD_AXIS_MOTION:
                    handle_axis_motion(event, contexts, deadzone_raw, trigger_threshold)
                elif event.type in (sdl3.SDL_EVENT_GAMEPAD_BUTTON_DOWN, sdl3.SDL_EVENT_GAMEPAD_BUTTON_UP):
                    handle_button_event(event, contexts, button_map)
                elif event.type == sdl3.SDL_EVENT_GAMEPAD_REMOVED:
                    ctx = contexts.get(event.gdevice.which)
                    if ctx:
                        console.print(f"[yellow]PC controller {ctx.pc_index} removed; sending neutral to slot {ctx.switch_slot}[/yellow]")
                        uart.send_report(SwitchReport(), controller_id=ctx.switch_slot)
            now = time.monotonic()
            for ctx in contexts.values():
                poll_controller_buttons(ctx, button_map)
                if now - ctx.last_send >= interval:
                    try:
                        uart.send_report(ctx.report, controller_id=ctx.switch_slot)
                    except SerialException:
                        raise
                    ctx.last_send = now
            # Drain rumble to avoid UART buffer growth. Rumble is still best-effort/shared in this V2.
            payload = uart.read_rumble_payload()
            if payload and contexts:
                target = next((c for c in contexts.values() if c.switch_slot == 0), next(iter(contexts.values())))
                try:
                    left, right = decode_rumble(payload)
                    strength = max(left, right)
                    if strength >= RUMBLE_MIN_ACTIVE:
                        sdl3.SDL_RumbleGamepad(target.controller, int(left * 0xFFFF), int(right * 0xFFFF), 10)
                    elif target.rumble_active:
                        sdl3.SDL_RumbleGamepad(target.controller, 0, 0, 0)
                    target.rumble_active = strength >= RUMBLE_MIN_ACTIVE
                    target.last_rumble = now
                except Exception:
                    pass
            sdl3.SDL_Delay(1)
    finally:
        for ctx in contexts.values():
            try:
                sdl3.SDL_RumbleGamepad(ctx.controller, 0, 0, 0)
                sdl3.SDL_CloseGamepad(ctx.controller)
            except Exception:
                pass
        if uart:
            uart.close()
        sdl3.SDL_Quit()


def main() -> None:
    # Preserve original behavior unless the new --port mode is explicitly requested.
    has_port = any(arg == "--port" or arg.startswith("--port=") for arg in sys.argv[1:])
    if not has_port:
        return legacy.main()
    parser = build_parser()
    args = parser.parse_args()
    console = Console()
    if args.list_controllers:
        if not sdl3.SDL_Init(sdl3.SDL_INIT_GAMEPAD | sdl3.SDL_INIT_JOYSTICK):
            parser.error("SDL init failed")
        try:
            list_controllers(console)
        finally:
            sdl3.SDL_Quit()
        return
    if not args.map:
        parser.error("at least one --map PC:SWITCH is required in --port mode")
    validate_maps(parser, args.map)
    run_multi(args)


if __name__ == "__main__":
    main()
