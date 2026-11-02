# NMOS 6502 interpreter adapted from H.A.V.O.C. (MIT; see ../LICENSE).
#!/usr/bin/env python3
"""Shared NMOS 6502 interpreter and ELF loader for linked-native regression tests.
No emulator audio or physical hardware timing is implied.
"""
import argparse
import json
import runpy
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RESIDENT_SECTIONS = (".text", ".rodata", ".tables", ".data", ".sfx_patches", ".sfx_helpers")
C, Z, I, D, B, U, V, N = (1 << n for n in range(8))
SENTINEL = 0xFFFF

# Addressing modes: (operand bytes, resolver name). Base cycle counts follow
# the NMOS 6502 data sheet; page-crossing and branch penalties are added below.
OPCODES = {
    0x69: ("ADC", "imm", 2), 0x65: ("ADC", "zp", 3), 0x75: ("ADC", "zpx", 4), 0x6D: ("ADC", "abs", 4),
    0x7D: ("ADC", "abx", 4), 0x79: ("ADC", "aby", 4), 0x61: ("ADC", "izx", 6), 0x71: ("ADC", "izy", 5),
    0x29: ("AND", "imm", 2), 0x25: ("AND", "zp", 3), 0x35: ("AND", "zpx", 4), 0x2D: ("AND", "abs", 4),
    0x3D: ("AND", "abx", 4), 0x39: ("AND", "aby", 4), 0x21: ("AND", "izx", 6), 0x31: ("AND", "izy", 5),
    0x0A: ("ASL", "acc", 2), 0x06: ("ASL", "zp", 5), 0x16: ("ASL", "zpx", 6), 0x0E: ("ASL", "abs", 6), 0x1E: ("ASL", "abx", 7),
    0x90: ("BCC", "rel", 2), 0xB0: ("BCS", "rel", 2), 0xF0: ("BEQ", "rel", 2), 0x30: ("BMI", "rel", 2),
    0xD0: ("BNE", "rel", 2), 0x10: ("BPL", "rel", 2), 0x50: ("BVC", "rel", 2), 0x70: ("BVS", "rel", 2),
    0x24: ("BIT", "zp", 3), 0x2C: ("BIT", "abs", 4), 0x00: ("BRK", "imp", 7),
    0x18: ("CLC", "imp", 2), 0xD8: ("CLD", "imp", 2), 0x58: ("CLI", "imp", 2), 0xB8: ("CLV", "imp", 2),
    0xC9: ("CMP", "imm", 2), 0xC5: ("CMP", "zp", 3), 0xD5: ("CMP", "zpx", 4), 0xCD: ("CMP", "abs", 4),
    0xDD: ("CMP", "abx", 4), 0xD9: ("CMP", "aby", 4), 0xC1: ("CMP", "izx", 6), 0xD1: ("CMP", "izy", 5),
    0xE0: ("CPX", "imm", 2), 0xE4: ("CPX", "zp", 3), 0xEC: ("CPX", "abs", 4),
    0xC0: ("CPY", "imm", 2), 0xC4: ("CPY", "zp", 3), 0xCC: ("CPY", "abs", 4),
    0xC6: ("DEC", "zp", 5), 0xD6: ("DEC", "zpx", 6), 0xCE: ("DEC", "abs", 6), 0xDE: ("DEC", "abx", 7),
    0xCA: ("DEX", "imp", 2), 0x88: ("DEY", "imp", 2),
    0x49: ("EOR", "imm", 2), 0x45: ("EOR", "zp", 3), 0x55: ("EOR", "zpx", 4), 0x4D: ("EOR", "abs", 4),
    0x5D: ("EOR", "abx", 4), 0x59: ("EOR", "aby", 4), 0x41: ("EOR", "izx", 6), 0x51: ("EOR", "izy", 5),
    0xE6: ("INC", "zp", 5), 0xF6: ("INC", "zpx", 6), 0xEE: ("INC", "abs", 6), 0xFE: ("INC", "abx", 7),
    0xE8: ("INX", "imp", 2), 0xC8: ("INY", "imp", 2),
    0x4C: ("JMP", "abs", 3), 0x6C: ("JMP", "ind", 5), 0x20: ("JSR", "abs", 6),
    0xA9: ("LDA", "imm", 2), 0xA5: ("LDA", "zp", 3), 0xB5: ("LDA", "zpx", 4), 0xAD: ("LDA", "abs", 4),
    0xBD: ("LDA", "abx", 4), 0xB9: ("LDA", "aby", 4), 0xA1: ("LDA", "izx", 6), 0xB1: ("LDA", "izy", 5),
    0xA2: ("LDX", "imm", 2), 0xA6: ("LDX", "zp", 3), 0xB6: ("LDX", "zpy", 4), 0xAE: ("LDX", "abs", 4), 0xBE: ("LDX", "aby", 4),
    0xA0: ("LDY", "imm", 2), 0xA4: ("LDY", "zp", 3), 0xB4: ("LDY", "zpx", 4), 0xAC: ("LDY", "abs", 4), 0xBC: ("LDY", "abx", 4),
    0x4A: ("LSR", "acc", 2), 0x46: ("LSR", "zp", 5), 0x56: ("LSR", "zpx", 6), 0x4E: ("LSR", "abs", 6), 0x5E: ("LSR", "abx", 7),
    0xEA: ("NOP", "imp", 2),
    0x09: ("ORA", "imm", 2), 0x05: ("ORA", "zp", 3), 0x15: ("ORA", "zpx", 4), 0x0D: ("ORA", "abs", 4),
    0x1D: ("ORA", "abx", 4), 0x19: ("ORA", "aby", 4), 0x01: ("ORA", "izx", 6), 0x11: ("ORA", "izy", 5),
    0x48: ("PHA", "imp", 3), 0x08: ("PHP", "imp", 3), 0x68: ("PLA", "imp", 4), 0x28: ("PLP", "imp", 4),
    0x2A: ("ROL", "acc", 2), 0x26: ("ROL", "zp", 5), 0x36: ("ROL", "zpx", 6), 0x2E: ("ROL", "abs", 6), 0x3E: ("ROL", "abx", 7),
    0x6A: ("ROR", "acc", 2), 0x66: ("ROR", "zp", 5), 0x76: ("ROR", "zpx", 6), 0x6E: ("ROR", "abs", 6), 0x7E: ("ROR", "abx", 7),
    0x40: ("RTI", "imp", 6), 0x60: ("RTS", "imp", 6),
    0xE9: ("SBC", "imm", 2), 0xE5: ("SBC", "zp", 3), 0xF5: ("SBC", "zpx", 4), 0xED: ("SBC", "abs", 4),
    0xFD: ("SBC", "abx", 4), 0xF9: ("SBC", "aby", 4), 0xE1: ("SBC", "izx", 6), 0xF1: ("SBC", "izy", 5),
    0x38: ("SEC", "imp", 2), 0xF8: ("SED", "imp", 2), 0x78: ("SEI", "imp", 2),
    0x85: ("STA", "zp", 3), 0x95: ("STA", "zpx", 4), 0x8D: ("STA", "abs", 4), 0x9D: ("STA", "abx", 5),
    0x99: ("STA", "aby", 5), 0x81: ("STA", "izx", 6), 0x91: ("STA", "izy", 6),
    0x86: ("STX", "zp", 3), 0x96: ("STX", "zpy", 4), 0x8E: ("STX", "abs", 4),
    0x84: ("STY", "zp", 3), 0x94: ("STY", "zpx", 4), 0x8C: ("STY", "abs", 4),
    0xAA: ("TAX", "imp", 2), 0xA8: ("TAY", "imp", 2), 0xBA: ("TSX", "imp", 2),
    0x8A: ("TXA", "imp", 2), 0x9A: ("TXS", "imp", 2), 0x98: ("TYA", "imp", 2),
}
OPERAND_BYTES = {"imp": 0, "acc": 0, "imm": 1, "zp": 1, "zpx": 1, "zpy": 1, "rel": 1,
                 "abs": 2, "abx": 2, "aby": 2, "ind": 2, "izx": 1, "izy": 1}


class KernelError(Exception):
    pass


class Cpu:
    """NMOS 6502 without decimal arithmetic (the compiler never sets D)."""

    def __init__(self, memory, writable):
        self.mem = memory
        self.writable = writable
        self.a = self.x = self.y = 0
        self.s = 0xFD
        self.p = U | I
        self.pc = 0
        self.cycles = 0
        self.writes = []

    def read(self, address):
        return self.mem[address & 0xFFFF]

    def write(self, address, value):
        address &= 0xFFFF
        if not any(start <= address < end for start, end in self.writable):
            raise KernelError(f"store outside the kernel's regions: ${address:04X} at PC ${self.pc:04X}")
        self.mem[address] = value & 0xFF
        self.writes.append(address)

    def push(self, value):
        self.mem[0x100 + self.s] = value & 0xFF
        self.s = (self.s - 1) & 0xFF

    def pop(self):
        self.s = (self.s + 1) & 0xFF
        return self.mem[0x100 + self.s]

    def nz(self, value):
        value &= 0xFF
        self.p = (self.p & ~(N | Z)) | (N if value & 0x80 else 0) | (Z if value == 0 else 0)
        return value

    def set(self, flag, on):
        self.p = (self.p | flag) if on else (self.p & ~flag)

    def operand(self, mode, raw):
        """Return (address or None, page_crossed)."""
        if mode in ("imp", "acc", "rel"):
            return None, False
        if mode == "imm":
            return self.pc - 1, False
        if mode == "zp":
            return raw, False
        if mode == "zpx":
            return (raw + self.x) & 0xFF, False
        if mode == "zpy":
            return (raw + self.y) & 0xFF, False
        if mode == "abs":
            return raw, False
        if mode == "abx":
            return (raw + self.x) & 0xFFFF, (raw & 0xFF00) != ((raw + self.x) & 0xFF00)
        if mode == "aby":
            return (raw + self.y) & 0xFFFF, (raw & 0xFF00) != ((raw + self.y) & 0xFF00)
        if mode == "ind":
            low = self.read(raw)
            high = self.read((raw & 0xFF00) | ((raw + 1) & 0xFF))
            return low | high << 8, False
        if mode == "izx":
            pointer = (raw + self.x) & 0xFF
            return self.read(pointer) | self.read((pointer + 1) & 0xFF) << 8, False
        if mode == "izy":
            base = self.read(raw) | self.read((raw + 1) & 0xFF) << 8
            return (base + self.y) & 0xFFFF, (base & 0xFF00) != ((base + self.y) & 0xFF00)
        raise KernelError(f"unsupported addressing mode {mode}")

    def branch(self, taken, raw):
        if taken:
            target = (self.pc + (raw - 256 if raw & 0x80 else raw)) & 0xFFFF
            self.cycles += 2 if (target & 0xFF00) != (self.pc & 0xFF00) else 1
            self.pc = target

    def adc(self, value):
        if self.p & D:
            raise KernelError("decimal arithmetic is not modelled")
        total = self.a + value + (1 if self.p & C else 0)
        self.set(V, bool(~(self.a ^ value) & (self.a ^ total) & 0x80))
        self.set(C, total > 0xFF)
        self.a = self.nz(total)

    def compare(self, register, value):
        self.set(C, register >= value)
        self.nz(register - value)

    def step(self):
        opcode = self.read(self.pc)
        if opcode not in OPCODES:
            raise KernelError(f"illegal opcode ${opcode:02X} at ${self.pc:04X}")
        name, mode, cycles = OPCODES[opcode]
        size = OPERAND_BYTES[mode]
        raw = 0
        if size == 1:
            raw = self.read(self.pc + 1)
        elif size == 2:
            raw = self.read(self.pc + 1) | self.read(self.pc + 2) << 8
        self.pc = (self.pc + 1 + size) & 0xFFFF
        self.cycles += cycles
        address, crossed = self.operand(mode, raw)
        if crossed and name in ("ADC", "AND", "CMP", "EOR", "LDA", "LDX", "LDY", "ORA", "SBC"):
            self.cycles += 1
        load = (lambda: self.a) if mode == "acc" else (lambda: self.read(address))

        def store(value):
            if mode == "acc":
                self.a = value & 0xFF
            else:
                self.write(address, value)

        if name == "ADC":
            self.adc(load())
        elif name == "SBC":
            self.adc(load() ^ 0xFF)
        elif name == "AND":
            self.a = self.nz(self.a & load())
        elif name == "ORA":
            self.a = self.nz(self.a | load())
        elif name == "EOR":
            self.a = self.nz(self.a ^ load())
        elif name == "ASL":
            value = load()
            self.set(C, bool(value & 0x80))
            store(self.nz(value << 1))
        elif name == "LSR":
            value = load()
            self.set(C, bool(value & 1))
            store(self.nz(value >> 1))
        elif name == "ROL":
            value = load()
            carry = 1 if self.p & C else 0
            self.set(C, bool(value & 0x80))
            store(self.nz((value << 1) | carry))
        elif name == "ROR":
            value = load()
            carry = 0x80 if self.p & C else 0
            self.set(C, bool(value & 1))
            store(self.nz((value >> 1) | carry))
        elif name == "BCC":
            self.branch(not self.p & C, raw)
        elif name == "BCS":
            self.branch(bool(self.p & C), raw)
        elif name == "BEQ":
            self.branch(bool(self.p & Z), raw)
        elif name == "BNE":
            self.branch(not self.p & Z, raw)
        elif name == "BMI":
            self.branch(bool(self.p & N), raw)
        elif name == "BPL":
            self.branch(not self.p & N, raw)
        elif name == "BVC":
            self.branch(not self.p & V, raw)
        elif name == "BVS":
            self.branch(bool(self.p & V), raw)
        elif name == "BIT":
            value = load()
            self.set(Z, self.a & value == 0)
            self.set(N, bool(value & 0x80))
            self.set(V, bool(value & 0x40))
        elif name == "BRK":
            raise KernelError(f"BRK executed at ${self.pc - 1:04X}")
        elif name in ("CLC", "CLD", "CLI", "CLV"):
            self.set({"CLC": C, "CLD": D, "CLI": I, "CLV": V}[name], False)
        elif name in ("SEC", "SED", "SEI"):
            self.set({"SEC": C, "SED": D, "SEI": I}[name], True)
        elif name == "CMP":
            self.compare(self.a, load())
        elif name == "CPX":
            self.compare(self.x, load())
        elif name == "CPY":
            self.compare(self.y, load())
        elif name == "DEC":
            store(self.nz(load() - 1))
        elif name == "INC":
            store(self.nz(load() + 1))
        elif name == "DEX":
            self.x = self.nz(self.x - 1)
        elif name == "DEY":
            self.y = self.nz(self.y - 1)
        elif name == "INX":
            self.x = self.nz(self.x + 1)
        elif name == "INY":
            self.y = self.nz(self.y + 1)
        elif name == "JMP":
            self.pc = address
        elif name == "JSR":
            back = (self.pc - 1) & 0xFFFF
            self.push(back >> 8)
            self.push(back & 0xFF)
            self.pc = address
        elif name == "RTS":
            low = self.pop()
            high = self.pop()
            self.pc = ((low | high << 8) + 1) & 0xFFFF
        elif name == "RTI":
            self.p = (self.pop() | U) & ~B
            low = self.pop()
            high = self.pop()
            self.pc = low | high << 8
        elif name == "LDA":
            self.a = self.nz(load())
        elif name == "LDX":
            self.x = self.nz(load())
        elif name == "LDY":
            self.y = self.nz(load())
        elif name == "STA":
            self.write(address, self.a)
        elif name == "STX":
            self.write(address, self.x)
        elif name == "STY":
            self.write(address, self.y)
        elif name == "NOP":
            pass
        elif name == "PHA":
            self.push(self.a)
        elif name == "PHP":
            self.push(self.p | B | U)
        elif name == "PLA":
            self.a = self.nz(self.pop())
        elif name == "PLP":
            self.p = (self.pop() | U) & ~B
        elif name == "TAX":
            self.x = self.nz(self.a)
        elif name == "TAY":
            self.y = self.nz(self.a)
        elif name == "TSX":
            self.x = self.nz(self.s)
        elif name == "TXA":
            self.a = self.nz(self.x)
        elif name == "TXS":
            self.s = self.x
        elif name == "TYA":
            self.a = self.nz(self.y)
        else:
            raise KernelError(f"unhandled instruction {name}")
