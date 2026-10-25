"""Alternate-EQ oracle (T04G/T04H): run WING-Edit's own eq_parchg_<model> in the emulator for arbitrary
parameter sets and dump the section coefficients plus every traced Cbiquad primitive call (exact float args).
Used by `WING_EQ_ALT_ORACLE=<out.json> cargo test -p wing-blocks oracle` (crates/wing-blocks/src/models/eq_alt).
Non-finite junk in unused argument registers is written as NaN; replace with 0 before loading in Rust.
usage: eq_alt_oracle.py <thin arm64 WING-Edit> <out.json> <jobs.json>   (thin: lipo -thin arm64 ...)
jobs from eq_alt_jobs.py <n> <seed> [models] [mix]
jobs: [{"model":..., "base":{...}, "sets":[{...},...]}]"""
import json
import os
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curvelib import Wing  # noqa: E402
import curves as C  # noqa: E402
from unicorn import UC_HOOK_CODE  # noqa: E402
from unicorn.arm64_const import (UC_ARM64_REG_X0, UC_ARM64_REG_X1, UC_ARM64_REG_X2, UC_ARM64_REG_X3,  # noqa: E402
                                 UC_ARM64_REG_S0, UC_ARM64_REG_S1, UC_ARM64_REG_S2, UC_ARM64_REG_S3,
                                 UC_ARM64_REG_S4, UC_ARM64_REG_LR)


def f32(v):
    return struct.unpack("<f", struct.pack("<I", v & 0xFFFFFFFF))[0]


def main():
    thin, out = sys.argv[1], sys.argv[2]
    jobs = json.load(open(sys.argv[3]))
    w = Wing(thin)
    e = w.e
    g = C.Gen(w)
    prims = {a: n for n, a in e.sym.items() if n.startswith("__ZN7Cbiquad")}
    calls = []

    def hook(uc, addr, size, data):
      try:
        if addr in prims:
            r = uc.reg_read
            calls.append({"fn": prims[addr].replace("__ZN7Cbiquad", ""),
                          "dst": r(UC_ARM64_REG_X0), "w1": r(UC_ARM64_REG_X1) & 0xFFFFFFFF,
                          "x2": r(UC_ARM64_REG_X2), "x3": r(UC_ARM64_REG_X3) & 0xFFFFFFFF,
                          "s": [f32(r(x)) for x in (UC_ARM64_REG_S0, UC_ARM64_REG_S1, UC_ARM64_REG_S2,
                                                     UC_ARM64_REG_S3, UC_ARM64_REG_S4)],
                          "lr": r(UC_ARM64_REG_LR)})
      except Exception as ex:
        print("hook error", ex, file=sys.stderr)
    lo, hi = min(prims), max(prims) + 0x1000
    res = []
    for job in jobs:
        mdl = job["model"]
        path = "bus/1/eq" if mdl == "PIA" else "ch/1/eq"
        w.set(path, "mdl", mdl)
        w.refresh(path)
        n = w.node(path)
        u = e.i32(n + C.EQ_USAGE)
        idx = e.i32(n + C.EQ_INDEX)
        m = w.ival(e.u64(n + C.EQ_MDL))
        fn = e.u64(g.eq_tab[u] + 0x40 * m + 0x30)
        names = [nm for nm, _ in w.children(path)]
        for s in job["sets"]:
            for k, v in job.get("base", {}).items():
                w.set(path, k, v)
            for k, v in s.items():
                w.set(path, k, v)
            calls.clear()
            h = e.uc.hook_add(UC_HOOK_CODE, hook, begin=lo, end=hi)
            e.uc.ctl_flush_tb()
            try:
                blk = w.node(path)
                e.call(w.vmethod(blk, "6updateEj"), blk, 0xFFFF)
            finally:
                e.uc.hook_del(h)
            _, reads = w.trace_reads(lambda: w.call_f(fn, idx, u, fl=(1000.0,)), *C.HEAP)
            addrs = sorted({a for a, sz in reads if sz == 4 and not (g.drv_lo <= a < g.drv_lo + 0x20)})
            secs = []
            for a in addrs[::5]:
                b2, b0, b1, a2, a1 = struct.unpack("<5f", e.rd(a, 20))
                secs.append({"addr": a, "bq": [b0, b1, b2, -a1, -a2]})
            pn = [x for x in names if not x.startswith('$') and '/' not in x]
            pv = g.params(path, pn)
            for x in pn:
                nd = w.node(f"{path}/{x}")
                if w.ftype(nd) in (1, 2) and isinstance(pv[x], float):
                    pv[x] = w.fval(nd)  # exact float32 value, not the 6-decimal display rounding
            res.append({"model": mdl, "set": s,
                        "params": pv,
                        "calls": list(calls), "sections": secs})
        w.set(path, "mdl", "STD")
    json.dump(res, open(out, "w"), indent=0)


main()
