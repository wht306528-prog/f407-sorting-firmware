#!/usr/bin/env python3
"""Minimal Modbus RTU servo simulator for the F407 USART2 motor bus.

It emulates two servo slaves enough for AppMotor_GotoAbsTargetAsRelative():
FC03 reads P0B-04/P0B-07, FC06 writes P0D-18/P0D-08/P0D-05, and FC16 writes
the 32-bit target pulse at P10-14. Use with an isolated USB-RS485 adapter; do
not put this simulator on a live bus with powered real drives.
"""

from __future__ import annotations

import argparse
import dataclasses
import struct
import sys
import time

try:
    import serial
except ImportError as exc:
    raise SystemExit("pyserial is required: python -m pip install pyserial") from exc


REG_ENABLE = 0x0D12
VAL_ENABLE_ON = 507
VAL_ENABLE_OFF = 511
REG_TRIGGER = 0x0D08
VAL_TRIGGER_ABS_MOVE = 3
VAL_TRIGGER_CLEAR = 0
REG_POS32_START = 0x100E
REG_FB_POS32_START = 0x0B07
REG_REACH_STATUS = 0x0B04
REG_ESTOP_MODE = 0x0D05


def crc16_modbus(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b
        for _ in range(8):
            if crc & 1:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc & 0xFFFF


def append_crc(pdu: bytes) -> bytes:
    crc = crc16_modbus(pdu)
    return pdu + bytes((crc & 0xFF, (crc >> 8) & 0xFF))


def regs_to_i32_lo_hi(reg_lo: int, reg_hi: int) -> int:
    u = (reg_lo & 0xFFFF) | ((reg_hi & 0xFFFF) << 16)
    return struct.unpack("<i", struct.pack("<I", u))[0]


def i32_to_regs_lo_hi(v: int) -> tuple[int, int]:
    u = struct.unpack("<I", struct.pack("<i", int(v)))[0]
    return u & 0xFFFF, (u >> 16) & 0xFFFF


@dataclasses.dataclass
class ServoState:
    slave: int
    current: int = 0
    target: int = 0
    enabled: bool = False
    estop: bool = False
    moving: bool = False
    reached: bool = True
    last_update: float = dataclasses.field(default_factory=time.monotonic)

    def update(self, speed_pulse_s: float) -> None:
        now = time.monotonic()
        dt = now - self.last_update
        self.last_update = now
        if not self.moving or not self.enabled or self.estop:
            return
        step = max(1, int(speed_pulse_s * dt))
        diff = self.target - self.current
        if abs(diff) <= step:
            self.current = self.target
            self.moving = False
            self.reached = True
        else:
            self.current += step if diff > 0 else -step
            self.reached = False

    def read_reg(self, addr: int, count: int) -> list[int]:
        if addr == REG_REACH_STATUS and count == 1:
            return [1 if self.reached else 0]
        if addr == REG_FB_POS32_START and count == 2:
            lo, hi = i32_to_regs_lo_hi(self.current)
            return [lo, hi]
        return [0] * count

    def write_single(self, addr: int, value: int) -> None:
        if addr == REG_ENABLE:
            self.enabled = value == VAL_ENABLE_ON
            if value == VAL_ENABLE_OFF:
                self.moving = False
        elif addr == REG_ESTOP_MODE:
            self.estop = value != 0
            if self.estop:
                self.enabled = False
                self.moving = False
        elif addr == REG_TRIGGER:
            if value == VAL_TRIGGER_ABS_MOVE and self.enabled and not self.estop:
                self.moving = self.current != self.target
                self.reached = not self.moving
            elif value == VAL_TRIGGER_CLEAR:
                self.moving = False

    def write_regs(self, addr: int, regs: list[int]) -> None:
        if addr == REG_POS32_START and len(regs) >= 2:
            self.target = regs_to_i32_lo_hi(regs[0], regs[1])
            self.reached = self.current == self.target


def hexdump(data: bytes) -> str:
    return " ".join(f"{b:02X}" for b in data)


class ServoSim:
    def __init__(self, port: str, baud: int, slaves: list[int], speed: float, verbose: bool) -> None:
        self.ser = serial.Serial(port=port, baudrate=baud, bytesize=8, parity="N", stopbits=1, timeout=0.02)
        self.slaves = {sid: ServoState(sid) for sid in slaves}
        self.speed = speed
        self.verbose = verbose
        self.buf = bytearray()

    def log(self, msg: str) -> None:
        if self.verbose:
            print(msg, flush=True)

    def run(self) -> None:
        print(f"servo sim listening on {self.ser.port} @ {self.ser.baudrate}, slaves={sorted(self.slaves)}")
        while True:
            for state in self.slaves.values():
                state.update(self.speed)
            chunk = self.ser.read(256)
            if chunk:
                self.buf.extend(chunk)
                self._drain()

    def _drain(self) -> None:
        while True:
            if len(self.buf) < 4:
                return
            fc = self.buf[1]
            if fc in (0x03, 0x06):
                need = 8
            elif fc == 0x10:
                if len(self.buf) < 7:
                    return
                need = 9 + self.buf[6]
            else:
                self.buf.pop(0)
                continue
            if len(self.buf) < need:
                return
            frame = bytes(self.buf[:need])
            del self.buf[:need]
            if crc16_modbus(frame[:-2]) != (frame[-2] | (frame[-1] << 8)):
                self.log(f"bad crc: {hexdump(frame)}")
                continue
            self._handle(frame)

    def _handle(self, frame: bytes) -> None:
        slave = frame[0]
        fc = frame[1]
        state = self.slaves.get(slave)
        if state is None:
            return
        state.update(self.speed)
        resp = b""
        if fc == 0x03:
            addr, count = struct.unpack(">HH", frame[2:6])
            regs = state.read_reg(addr, count)
            payload = bytearray([slave, fc, count * 2])
            for reg in regs:
                payload.extend(struct.pack(">H", reg & 0xFFFF))
            resp = append_crc(bytes(payload))
            self.log(f"RX {hexdump(frame)} -> read 0x{addr:04X}/{count}, pos={state.current}, reached={state.reached}")
        elif fc == 0x06:
            addr, value = struct.unpack(">HH", frame[2:6])
            state.write_single(addr, value)
            resp = frame
            self.log(f"RX {hexdump(frame)} -> write 0x{addr:04X}=0x{value:04X}, en={state.enabled}, estop={state.estop}")
        elif fc == 0x10:
            addr, count = struct.unpack(">HH", frame[2:6])
            regs = [struct.unpack(">H", frame[7 + i * 2 : 9 + i * 2])[0] for i in range(count)]
            state.write_regs(addr, regs)
            resp = append_crc(frame[:6])
            self.log(f"RX {hexdump(frame)} -> write regs 0x{addr:04X}/{count}, target={state.target}")
        if resp:
            self.ser.write(resp)
            self.log(f"TX {hexdump(resp)}")


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(description="F407 servo Modbus RTU simulator")
    parser.add_argument("--port", required=True, help="USB-RS485 serial port, e.g. COM7")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--slaves", default="1,2", help="comma-separated slave IDs")
    parser.add_argument("--speed", type=float, default=80000.0, help="simulated pulse speed per second")
    parser.add_argument("-v", "--verbose", action="store_true")
    args = parser.parse_args(argv)
    slaves = [int(x, 0) for x in args.slaves.split(",") if x.strip()]
    ServoSim(args.port, args.baud, slaves, args.speed, args.verbose).run()
    return 0


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
