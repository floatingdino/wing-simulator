#!/usr/bin/env python3
"""T04J: WING-Edit's own filter / EQ coefficients at fs = 44.1 kHz (small grids, including EQ `mix`).

Usage:
  <venv>/bin/python -I tools/curve-extract/curves44k1.py (--app WING-Edit.app | --thin <arm64 slice>) \
      --out data/reference-curves/44k1

Same method as curves.py (docs/research/reference-curves.md), except that the dummy audio engine's sample rate
(`_AEDRV+8` = fs, `_AEDRV+0x10` = pi/fs as float32) is patched to 44.1 kHz before any block computes
coefficients. Every coefficient routine (Cbiquad primitives, eq_parchg_*, Cchfilter, the hptab44/hptab48 switch)
reads these two fields on each call, so the traced biquads are what WING-Edit computes at 44.1 kHz.
For every curve the full parameter set of the block is recorded (exact float32 values), so curves are
independent of order. Output: filter.json, eq-std-{ch,aux,bus}.json, eq-alt.json.
"""
import argparse
import hashlib
import json
import math
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curvelib import (Wing, thin_binary, freq_grid, biquad_response, biquad_response_app, G_AE, G_AEDRV,  # noqa: E402
                      AE_CHSTRUCT, EQ_USAGE, EQ_INDEX, EQ_MDL)
import curves as C  # noqa: E402

FS = 44100.0
FREQS = freq_grid(20.0, 20000.0, 6)


def set_fs(w, fs):
    e = w.e
    drv = e.u64(G_AEDRV)
    e.uc.mem_write(drv + 8, struct.pack("<f", fs))
    e.uc.mem_write(drv + 0x10, struct.pack("<f", math.pi / fs))
    assert e.f32(drv + 8) == fs


class Gen44(C.Gen):
    def exact_params(self, path):
        w = self.w
        names = [nm for nm, _ in w.children(path) if not nm.startswith("$") and "/" not in nm]
        pv = {}
        for nm in names:
            n = w.node(f"{path}/{nm}")
            v = w.value(n)
            if w.ftype(n) in (1, 2) and isinstance(v, float):
                v = w.fval(n)
            pv[nm] = v
        return pv

    def capture(self, app_fn, coefs_fn=None, parallel=False):
        first, reads = self.w.trace_reads(lambda: app_fn(FREQS[0]), *C.HEAP)
        app = [first] + [app_fn(f) for f in FREQS[1:]]
        bq = coefs_fn() if coefs_fn else self.coef_groups(reads)
        assert bq, "no cascade traced"
        if app_fn is not None and not coefs_fn:
            dev = max(abs(a - m) for a, m in zip(app, biquad_response_app(bq, FREQS, fs=FS, parallel=parallel)))
            assert dev < 0.005, f"traced cascade disagrees with respf ({dev} dB)"
        mag, _ = biquad_response(bq, FREQS, fs=FS, parallel=parallel)
        out = {"mag_db": [round(x, 4) for x in mag], "biquads": self.std_form(bq)}
        if parallel:
            out["topology"] = "parallel"
        return out

    def run(self, path, settings, case, cap):
        w = self.w
        for k, v in settings.items():
            w.set(path, k, v)
        blk = w.node(path)
        e = self.e
        e.call(w.vmethod(blk, "6updateEj"), blk, 0xFFFF)
        rec = {"case": case, "params": self.exact_params(path)}
        rec.update(cap())
        return rec

    # ---- EQ
    def eq_cap(self, path):
        w, e = self.w, self.e
        n = w.node(path)
        u, idx = e.i32(n + EQ_USAGE), e.i32(n + EQ_INDEX)
        m = w.ival(e.u64(n + EQ_MDL))
        fn = e.u64(self.eq_tab[u] + 0x40 * m + 0x30)
        par = "maag4" in e.addr2sym.get(fn, "") and u == 0
        return lambda: self.capture(lambda f: w.call_f(fn, idx, u, fl=(f,)), parallel=par)

    def std(self, path, bands, lh_types, cuts):
        w = self.w
        w.set(path, "mdl", "STD")
        w.refresh(path)
        cap = self.eq_cap(path)
        names = [nm for nm, _ in w.children(path)]
        neutral = {f"{b}g": 0.0 for b in bands}
        neutral.update({"leq": "SHV", "heq": "SHV", "mix": 100.0})
        if "tilt" in names:
            neutral["tilt"] = 0.0
        out = []

        def go(case, **kv):
            out.append(self.run(path, dict(neutral, **kv), case, cap))
        for g in (-12, 6):
            for f in (40, 1000, 12000, 19000):
                for q in (0.7, 5):
                    go("mid-peq", **{"2g": g, "2f": f, "2q": q})
        for b in ("l", "h"):
            for t in lh_types:
                for f in (60, 1000, 16000):
                    go(f"{b}-{t}", **{f"{b}eq": t, f"{b}g": -9 if t == "PEQ" else 9, f"{b}f": f, f"{b}q": 1})
            for t in cuts:
                for f in ((60, 1000, 16000) if t == "CUT" else (1000,)):
                    for q in ((0.707, 2) if t == "CUT" else (1,)):
                        go(f"{b}-{t}", **{f"{b}eq": t, f"{b}f": f, f"{b}q": q})
        if "tilt" in neutral:
            for t in (-3, 3):
                go("tilt", tilt=t)
        # mix law: every mid band boosted/cut, mix 0..125 %
        mids = {}
        for i, b in enumerate(x for x in bands if x not in ("l", "h")):
            mids.update({f"{b}g": [9, -6, 12, -15, 4, -3][i], f"{b}f": [80, 300, 1200, 4000, 9000, 15000][i],
                         f"{b}q": [0.7, 2, 4, 1, 3, 0.5][i]})
        for mix in (0, 25, 50, 125):
            go("mid-mix", mix=mix, **mids)
        for b in ("l", "h"):
            for mix in (50, 125):
                go(f"{b}-SHV", mix=mix, **{f"{b}eq": "SHV", f"{b}g": 12, f"{b}f": 200 if b == "l" else 6000,
                                           f"{b}q": 1})
        go("flat")
        w.set(path, "mix", 100.0)
        return {"block": path, "model": "STD", "curves": out}

    # ---- alternate EQs: random settings (eq_alt_jobs.py generators), mix cycling 100/50/125/0
    def alt(self, per_model=6, seed=44100):
        import contextlib
        import io
        argv, sys.argv = sys.argv, [sys.argv[0], "0", str(seed)]
        with contextlib.redirect_stdout(io.StringIO()):
            import eq_alt_jobs  # module prints jobs for sys.argv; we only need its generators
        sys.argv = argv
        random.seed(seed)
        out = []
        for mdl, gens in eq_alt_jobs.M.items():
            path = "bus/1/eq" if mdl == "PIA" else "ch/1/eq"
            self.w.set(path, "mdl", mdl)
            self.w.refresh(path)
            cap = self.eq_cap(path)
            for i in range(per_model):
                s = {k: f() for k, f in gens.items()}
                s["mix"] = [100.0, 50.0, 125.0, 0.0, 75.0, 110.0][i % 6]
                out.append(dict(self.run(path, s, mdl, cap), model=mdl, block=path))
            self.w.set(path, "mix", 100.0)
            self.w.set(path, "mdl", "STD")
        return {"curves": out}

    # ---- channel filter (LC/HC + tool)
    def filt(self):
        w, e = self.w, self.e
        path = "ch/1/flt"
        w.refresh(path)
        blk = w.node(path)
        ch = e.u64(e.u64(e.u64(G_AE) + AE_CHSTRUCT) + 8 * e.i32(blk + 0xF8))

        def cap():
            bq = [struct.unpack("<5f", e.rd(ch + 0x12C + 20 * i, 20)) for i in range(4)]
            mag, _ = biquad_response(bq, FREQS, fs=FS)
            return {"mag_db": [round(x, 4) for x in mag], "biquads": self.std_form(bq)}
        off = {"lc": 0, "hc": 0, "tf": 0}
        out = [self.run(path, off, "flat", cap)]
        for s in ("6", "12", "18", "24"):
            for f in (20, 100, 1000):
                out.append(self.run(path, dict(off, lc=1, lcs=s, lcf=f), f"lc-{s}", cap))
        for s in ("6", "12"):
            for f in (1000, 10000, 20000):
                out.append(self.run(path, dict(off, hc=1, hcs=s, hcf=f), f"hc-{s}", cap))
        out.append(self.run(path, dict(off, lc=1, lcs="24", lcf=100, hc=1, hcs="12", hcf=8000), "lc+hc", cap))
        tool = [("TILT", [{"tilt": -6}, {"tilt": 3}]), ("MAX", [{"low": 25, "proc": 75}, {"low": 100, "proc": 50}]),
                ("AP1", [{"f": 100}, {"f": 5000}]),
                ("AP2", [{"f": 300, "q": 0.5}, {"f": 5000, "q": 5}])]
        for mdl, sets in tool:
            w.set(path, "mdl", mdl)
            for s in sets:
                out.append(self.run(path, dict(off, tf=1, **s), f"tool-{mdl}", cap))
        w.set(path, "mdl", "TILT")
        w.set(path, "tf", 0)
        return {"block": path, "curves": out}


def dump(obj, path, header):
    obj = dict(header, **obj)
    with open(path, "w") as f:
        json.dump(obj, f, separators=(",", ":"))
    print(f"wrote {path} ({os.path.getsize(path) / 1e3:.0f} kB, {len(obj['curves'])} curves)", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--app")
    ap.add_argument("--thin")
    ap.add_argument("--sha256", help="sha256 of the universal executable (when only --thin is given)")
    ap.add_argument("--out", default="data/reference-curves/44k1")
    a = ap.parse_args()
    if a.app:
        exe, thin = thin_binary(a.app)
        sha = hashlib.sha256(open(exe, "rb").read()).hexdigest()
    else:
        thin, sha = a.thin, a.sha256
    w = Wing(thin)
    set_fs(w, FS)
    g = Gen44(w)
    os.makedirs(a.out, exist_ok=True)
    hdr = {"format": "wing-reference-curves/1",
           "source": {"app": "WING-Edit", "sha256": sha, "slice": "arm64",
                      "generator": "tools/curve-extract/curves44k1.py"},
           "fs": FS, "freq_hz": FREQS,
           "conventions": {
               "mag_db": "20*log10|H| of the traced biquads at fs (double precision; 'topology: parallel' = sum)",
               "biquads": "[b0,b1,b2,a1,a2] float32 as computed by WING-Edit with _AEDRV fs patched to 44100",
               "params": "every parameter of the block after the set (floats as exact float32 values)"}}
    dump(g.filt(), os.path.join(a.out, "filter.json"), hdr)
    shv = ["PEQ", "SHV"]
    cuts = ["CUT", "BW6", "BW12", "BS12", "LR12", "BW18", "BW24", "BS24", "LR24", "BW48", "LR48"]
    dump(g.std("ch/1/eq", ["l", "1", "2", "3", "4", "h"], shv, []), os.path.join(a.out, "eq-std-ch.json"), hdr)
    dump(g.std("aux/1/eq", ["l", "1", "2", "3", "4", "h"], [], ["CUT"]), os.path.join(a.out, "eq-std-aux.json"), hdr)
    dump(g.std("bus/1/eq", ["l", "1", "2", "3", "4", "5", "6", "h"], shv, cuts),
         os.path.join(a.out, "eq-std-bus.json"), hdr)
    dump(g.alt(), os.path.join(a.out, "eq-alt.json"), hdr)


if __name__ == "__main__":
    main()
