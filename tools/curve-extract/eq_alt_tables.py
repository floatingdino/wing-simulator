"""Generate Rust table constants from the WING-Edit parchg static tables.
usage: eq_alt_tables.py <thin> > crates/wing-blocks/src/models/eq_alt/tables.rs"""
import bisect
import re
import struct
import sys

import lief

b = lief.parse(sys.argv[1])
syms = {}
for s in b.symbols:
    if s.value and s.name:
        syms.setdefault(s.value, s.name)
addrs = sorted(syms)


def read(a, n):
    for sec in b.sections:
        if sec.virtual_address <= a < sec.virtual_address + sec.size:
            off = a - sec.virtual_address
            return bytes(sec.content[off:off + n])


def f32s(x):
    s = "%.9g" % x
    for p in range(6, 10):
        t = "%.*g" % (p, x)
        if struct.unpack("<f", struct.pack("<f", float(t)))[0] == x:
            s = t
            break
    if "e" in s:
        m, e = s.split("e")
        if "." not in m:
            m += ".0"
        s = f"{m}e{int(e)}"
    elif "." not in s and "inf" not in s and "nan" not in s:
        s += ".0"
    return s


MODELS = [("ssl_g", "SSL"), ("neve88rs", "N88"), ("neve84", "N84"), ("pultec_duo", "PUL"), ("api560", "API")]
print("//! Coefficient tables of the WING-Edit 3.3.4 `eq_parchg_*` functions (static locals), extracted verbatim")
print("//! (float32 bit patterns) from the arm64 slice by the T04G/T04H agent. Generated; do not edit by hand.")
print("#![allow(clippy::excessive_precision, clippy::approx_constant, clippy::unreadable_literal)]")
print()
for key, pre in MODELS:
    for a in addrs:
        nm = syms[a]
        if f"eq_parchg_{key}E" in nm and nm.startswith("__ZZN") and not re.search(r"\.\d+$", nm) is None:
            pass
    seen = set()
    for a in addrs:
        nm = syms[a]
        if f"eq_parchg_{key}E" in nm and nm.startswith("__ZZN"):
            m = re.search(r"Cbiquad[E]\d+([A-Za-z0-9_]+)\.\d+$", nm)
            if not m:
                continue
            tname = m.group(1)
            if tname in seen:
                continue
            seen.add(tname)
            j = bisect.bisect_right(addrs, a)
            size = addrs[j] - a
            if pre == "N84" and tname == "LoShvTab":
                size = 20 * 196  # the symbol span also covers isa110's anonymous tables (RAW below)
            raw = read(a, size)
            n = size // 4
            fl = struct.unpack(f"<{n}f", raw[:n * 4])
            rname = f"{pre}_{tname.upper()}"
            print(f"/// `{tname}` of `eq_parchg_{key}` ({n} values, at {a:#x}).")
            print(f"pub(super) static {rname}: [f32; {n}] = [")
            for i in range(0, n, 8):
                print("    " + ", ".join(f32s(x) for x in fl[i:i + 8]) + ",")
            print("];")
            print()

# Anonymous tables referenced by address only (no symbol of their own).
RAW = [
    ("ISA_HF_F", 0x100621a6c, 54, "`eq_parchg_isa110` HF shelf f, 6 selectors x 9 gain columns"),
    ("ISA_HF_G", 0x100621b44, 54, "`eq_parchg_isa110` HF shelf gain"),
    ("ISA_HF_Q1", 0x100621c1c, 54, "`eq_parchg_isa110` HF shelf q1"),
    ("ISA_HF_Q2", 0x100621cf4, 54, "`eq_parchg_isa110` HF shelf q2"),
    ("ISA_LF", 0x100621dcc, 336,
     "`eq_parchg_isa110` LF, 6 selectors x 7 gain columns x (f, g, q1, q2, f2, g2, q1b, q2b)"),
]
for name, a, n, doc in RAW:
    raw = read(a, n * 4)
    fl = struct.unpack(f"<{n}f", raw)
    print(f"/// {doc} ({n} values, at {a:#x}).")
    print(f"pub(super) static {name}: [f32; {n}] = [")
    for i in range(0, n, 8):
        print("    " + ", ".join(f32s(x) for x in fl[i:i + 8]) + ",")
    print("];")
    print()
