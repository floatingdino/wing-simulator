"""Minimal arm64 Mach-O emulation harness for WING-Edit (static analysis aid).

The WING-Edit binary is never executed natively, modified, or attached to a
debugger. We load the arm64 slice's segments into a Unicorn CPU emulator,
replace every imported symbol with a Python stub, and call individual
internal functions (tree constructors, describe routines) to recover the
parameter tree exactly as the app builds it in memory.

Run with:  <venv>/bin/python -I wingemu.py  (see extract.py)
"""
import math
import struct
import subprocess
import sys

import lief
from unicorn import Uc, UcError, UC_ARCH_ARM64, UC_MODE_ARM, UC_HOOK_CODE, \
    UC_HOOK_MEM_UNMAPPED, UC_PROT_ALL
from unicorn.arm64_const import *

HEAP_BASE = 0x2_0000_0000
HEAP_SIZE = 0x4000_0000          # 1 GiB
STACK_BASE = 0x3_0000_0000
STACK_SIZE = 0x0100_0000         # 16 MiB
TRAP_BASE = 0x4_0000_0000
TRAP_SLOT = 0x40                 # bytes per import (also used as tiny data blob)
RET = struct.pack("<I", 0xD65F03C0)


class Emu:
    def __init__(self, macho_path, verbose=False):
        self.verbose = verbose
        fat = lief.MachO.parse(macho_path)
        self.bin = None
        for i in range(fat.size):
            b = fat.at(i)
            if b.header.cpu_type == lief.MachO.Header.CPU_TYPE.ARM64:
                self.bin = b
        assert self.bin is not None, "no arm64 slice"
        self.uc = Uc(UC_ARCH_ARM64, UC_MODE_ARM)
        try:  # enable LSE atomics etc.
            from unicorn.arm64_const import UC_CPU_ARM64_MAX
            self.uc.ctl_set_cpu_model(UC_CPU_ARM64_MAX)
        except Exception:
            pass
        raw = open(macho_path, "rb").read()
        self._map_segments()
        self.uc.mem_map(HEAP_BASE, HEAP_SIZE, UC_PROT_ALL)
        self.uc.mem_map(STACK_BASE, STACK_SIZE, UC_PROT_ALL)
        self.heap_ptr = HEAP_BASE + 0x1000
        self.allocs = {}
        # symbols (internal)
        self.sym = {}
        self.addr2sym = {}
        for s in self.bin.symbols:
            if s.value and s.name:
                self.sym[s.name] = s.value
                self.addr2sym.setdefault(s.value, s.name)
        self._bind_imports()
        self.uc.hook_add(UC_HOOK_CODE, self._trap_hook, begin=TRAP_BASE,
                         end=TRAP_BASE + self.trap_size)
        self.uc.hook_add(UC_HOOK_MEM_UNMAPPED, self._unmapped)
        self.unknown_calls = {}
        self.stop_reason = None

    # ------------------------------------------------------------------ load
    def _map_segments(self):
        for seg in self.bin.segments:
            if seg.name == "__PAGEZERO" or seg.virtual_size == 0:
                continue
            base = seg.virtual_address & ~0xFFF
            size = (seg.virtual_address + seg.virtual_size - base + 0xFFF) & ~0xFFF
            self.uc.mem_map(base, size, UC_PROT_ALL)
            data = bytes(seg.content)
            if data:
                self.uc.mem_write(seg.virtual_address, data)

    def _bind_imports(self):
        names = sorted({b.symbol.name for b in self.bin.bindings if b.has_symbol})
        self.trap_size = (len(names) + 16) * TRAP_SLOT
        self.trap_size = (self.trap_size + 0xFFF) & ~0xFFF
        self.uc.mem_map(TRAP_BASE, self.trap_size, UC_PROT_ALL)
        self.uc.mem_write(TRAP_BASE, RET * (self.trap_size // 4))
        self.imp_addr = {}
        self.addr_imp = {}
        for i, n in enumerate(names):
            a = TRAP_BASE + (i + 16) * TRAP_SLOT
            self.imp_addr[n] = a
            self.addr_imp[a] = n
        # data-like imports: give them a zeroed blob instead of RET bytes
        for n, a in self.imp_addr.items():
            if not n.startswith("_") or n in ("___stack_chk_guard",) or \
                    n.startswith("_k") or n.startswith("_NS") or n.startswith("__ZTV") or \
                    n.startswith("__ZTI") or n.startswith("_OBJC_"):
                self.uc.mem_write(a, b"\0" * TRAP_SLOT)
        for b in self.bin.bindings:
            if not b.has_symbol:
                continue
            self.uc.mem_write(b.address, struct.pack("<Q", self.imp_addr[b.symbol.name] + b.addend))
        self.SENTINEL = TRAP_BASE  # slot 0: return-to-python

    # --------------------------------------------------------------- helpers
    def r(self, reg):
        return self.uc.reg_read(reg)

    def w(self, reg, v):
        self.uc.reg_write(reg, v)

    def rd(self, addr, n):
        return bytes(self.uc.mem_read(addr, n))

    def u64(self, a):
        return struct.unpack("<Q", self.rd(a, 8))[0]

    def u32(self, a):
        return struct.unpack("<I", self.rd(a, 4))[0]

    def i32(self, a):
        return struct.unpack("<i", self.rd(a, 4))[0]

    def f32(self, a):
        return struct.unpack("<f", self.rd(a, 4))[0]

    def u8(self, a):
        return self.rd(a, 1)[0]

    def cstr(self, a, maxlen=4096):
        if not a:
            return None
        out = bytearray()
        while len(out) < maxlen:
            chunk = self.rd(a + len(out), 64)
            z = chunk.find(b"\0")
            if z >= 0:
                out += chunk[:z]
                break
            out += chunk
        return out.decode("latin-1")

    def malloc(self, n, zero=True):
        n = max(int(n), 1)
        p = (self.heap_ptr + 15) & ~15
        self.heap_ptr = p + n + 16
        if self.heap_ptr > HEAP_BASE + HEAP_SIZE:
            raise MemoryError("emulated heap exhausted")
        self.allocs[p] = n
        return p

    def wstr(self, s):
        b = s.encode("latin-1") + b"\0"
        p = self.malloc(len(b))
        self.uc.mem_write(p, b)
        return p

    # ------------------------------------------------------------------ call
    def call(self, func, *args, fargs=(), max_insns=0, timeout_us=0):
        if isinstance(func, str):
            func = self.sym[func]
        sp = STACK_BASE + STACK_SIZE - 0x1000
        self.w(UC_ARM64_REG_SP, sp)
        regs = [UC_ARM64_REG_X0 + i for i in range(8)]
        for i, a in enumerate(args):
            self.w(regs[i], a & 0xFFFFFFFFFFFFFFFF)
        for i, f in enumerate(fargs):
            self.w(UC_ARM64_REG_D0 + i, struct.unpack("<Q", struct.pack("<d", f))[0])
        self.w(UC_ARM64_REG_LR, self.SENTINEL)
        self.w(UC_ARM64_REG_X29, 0)
        self.stop_reason = None
        try:
            self.uc.emu_start(func, self.SENTINEL, timeout=timeout_us, count=max_insns)
        except UcError as e:
            pc = self.r(UC_ARM64_REG_PC)
            raise RuntimeError(f"emulation error {e} at pc={pc:#x} "
                               f"({self.near_sym(pc)}) reason={self.stop_reason}")
        if self.stop_reason:
            raise RuntimeError(self.stop_reason)
        return self.r(UC_ARM64_REG_X0)

    def near_sym(self, pc):
        best = None
        for a, n in self.addr2sym.items():
            if a <= pc and (best is None or a > best[0]):
                best = (a, n)
        return f"{best[1]}+{pc - best[0]:#x}" if best else "?"

    def _unmapped(self, uc, access, addr, size, value, data):
        pc = self.r(UC_ARM64_REG_PC)
        self.stop_reason = f"unmapped access {addr:#x} at pc={pc:#x}"
        return False

    # ----------------------------------------------------------------- stubs
    def _ret(self, v):
        self.w(UC_ARM64_REG_X0, v & 0xFFFFFFFFFFFFFFFF)

    def _fret(self, v, single):
        if single:
            self.w(UC_ARM64_REG_S0, struct.unpack("<I", struct.pack("<f", v))[0])
        else:
            self.w(UC_ARM64_REG_D0, struct.unpack("<Q", struct.pack("<d", v))[0])

    def _farg(self, i, single):
        if single:
            return struct.unpack("<f", struct.pack("<I", self.r(UC_ARM64_REG_S0 + i) & 0xFFFFFFFF))[0]
        return struct.unpack("<d", struct.pack("<Q", self.r(UC_ARM64_REG_D0 + i)))[0]

    def _trap_hook(self, uc, addr, size, data):
        if addr == self.SENTINEL:
            return
        name = self.addr_imp.get(addr)
        if name is None:
            return
        x = [self.r(UC_ARM64_REG_X0 + i) for i in range(4)]
        n = name[1:]
        if n in ("_Znwm", "_Znam", "malloc", "_ZnwmRKSt9nothrow_t"):
            self._ret(self.malloc(x[0]))
        elif n == "calloc":
            self._ret(self.malloc(x[0] * x[1]))
        elif n == "realloc":
            old = x[0]
            p = self.malloc(x[1])
            if old:
                self.uc.mem_write(p, self.rd(old, min(self.allocs.get(old, x[1]), x[1])))
            self._ret(p)
        elif n in ("_ZdlPv", "_ZdaPv", "free"):
            pass
        elif n == "memset":
            if x[2]:
                self.uc.mem_write(x[0], bytes([x[1] & 0xFF]) * x[2])
            self._ret(x[0])
        elif n == "bzero":
            if x[1]:
                self.uc.mem_write(x[0], b"\0" * x[1])
        elif n == "memset_pattern16":
            pat = self.rd(x[1], 16)
            self.uc.mem_write(x[0], (pat * (x[2] // 16 + 1))[:x[2]])
        elif n in ("memcpy", "memmove"):
            if x[2]:
                self.uc.mem_write(x[0], self.rd(x[1], x[2]))
            self._ret(x[0])
        elif n == "memcmp":
            a, b = self.rd(x[0], x[2]), self.rd(x[1], x[2])
            self._ret(0 if a == b else (-1 if a < b else 1))
        elif n == "memchr":
            buf = self.rd(x[0], x[2])
            i = buf.find(bytes([x[1] & 0xFF]))
            self._ret(0 if i < 0 else x[0] + i)
        elif n == "strlen":
            self._ret(len(self.cstr(x[0], 1 << 20).encode("latin-1")))
        elif n in ("strcmp", "strncmp"):
            a = self.cstr(x[0]) or ""
            b = self.cstr(x[1]) or ""
            if n == "strncmp":
                a, b = a[:x[2]], b[:x[2]]
            self._ret(0 if a == b else (-1 if a < b else 1))
        elif n == "strchr":
            s = self.cstr(x[0])
            c = chr(x[1] & 0xFF)
            if c == "\0":
                self._ret(x[0] + len(s))
            else:
                i = s.find(c)
                self._ret(0 if i < 0 else x[0] + i)
        elif n in ("strcpy", "strncpy"):
            s = self.cstr(x[1]).encode("latin-1") + b"\0"
            if n == "strncpy":
                s = (s + b"\0" * x[2])[:x[2]]
            self.uc.mem_write(x[0], s)
            self._ret(x[0])
        elif n == "atoi":
            try:
                self._ret(int((self.cstr(x[0]) or "0").strip().split()[0]))
            except Exception:
                self._ret(0)
        elif n in ("__toupper", "toupper"):
            self._ret(ord(chr(x[0]).upper()) if x[0] < 128 else x[0])
        elif n in ("__tolower", "tolower"):
            self._ret(ord(chr(x[0]).lower()) if x[0] < 128 else x[0])
        elif n == "__cxa_guard_acquire":
            g = x[0]
            self._ret(0 if self.u8(g) else 1)
        elif n == "__cxa_guard_release":
            self.uc.mem_write(x[0], b"\1")
        elif n in ("powf", "pow"):
            s = n.endswith("f")
            try:
                self._fret(math.pow(self._farg(0, s), self._farg(1, s)), s)
            except Exception:
                self._fret(float("nan"), s)
        elif n in ("expf", "exp", "logf", "log", "log10f", "log10", "exp2f", "exp2",
                   "cosf", "cos", "sinf", "sin", "atanf", "atan", "acos", "__exp10"):
            s = n.endswith("f")
            base = n.rstrip("f") if s else n
            fn = {"exp": math.exp, "log": math.log, "log10": math.log10,
                  "exp2": lambda v: 2.0 ** v, "cos": math.cos, "sin": math.sin,
                  "atan": math.atan, "acos": math.acos,
                  "__exp10": lambda v: 10.0 ** v}[base]
            try:
                self._fret(fn(self._farg(0, s)), s)
            except Exception:
                self._fret(float("nan"), s)
        elif n in ("atan2f", "atan2", "hypotf", "hypot", "fmod"):
            s = n.endswith("f")
            base = n.rstrip("f") if s else n
            fn = {"atan2": math.atan2, "hypot": math.hypot, "fmod": math.fmod}[base]
            try:
                self._fret(fn(self._farg(0, s), self._farg(1, s)), s)
            except Exception:
                self._fret(float("nan"), s)
        elif n == "ldexp":
            self._fret(math.ldexp(self._farg(0, False), x[0] & 0xFFFFFFFF), False)
        elif n == "__sincosf_stret":
            v = self._farg(0, True)
            self._fret(math.sin(v), True)
            self.w(UC_ARM64_REG_S1, struct.unpack("<I", struct.pack("<f", math.cos(v)))[0])
        elif n == "__sincos_stret":
            v = self._farg(0, False)
            self._fret(math.sin(v), False)
            self.w(UC_ARM64_REG_D1, struct.unpack("<Q", struct.pack("<d", math.cos(v)))[0])
        elif n in ("__stack_chk_fail", "abort", "__cxa_throw", "_Unwind_Resume", "exit",
                   "_ZSt9terminatev", "__cxa_pure_virtual"):
            self.stop_reason = f"fatal import {n} called (lr={self.r(UC_ARM64_REG_LR):#x} " \
                               f"{self.near_sym(self.r(UC_ARM64_REG_LR))})"
            self.uc.emu_stop()
        elif n == "__chkstk_darwin":
            pass
        else:
            self.unknown_calls[n] = self.unknown_calls.get(n, 0) + 1
            if n in ("pthread_mutex_init", "pthread_mutexattr_init"):
                self._ret(0)
            else:
                self._ret(0)
