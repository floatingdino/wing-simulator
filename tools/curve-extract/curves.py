#!/usr/bin/env python3
"""Export WING-Edit's own filter/EQ response curves and dynamics static curves as JSON oracles.

Usage:
  <venv>/bin/python -I tools/curve-extract/curves.py --app WING-Edit.app --out data/reference-curves \
      [--only filter,eq-std,dyn,sc,eq-alt,meter]

Method (docs/research/reference-curves.md): the parameter tree is built inside the Unicorn
emulator (T003 harness), parameters are set with ncl::Cpar::set/setInt, the block's own
update_direct()/update() compute the DSP biquads exactly as WING-Edit does (parchg), then

  * EQ:      the model's ae::eq_respf_<model>(idx, float f, usage) is called for every grid
             frequency (= magnitude the GUI draws, float, clamped to +-48 dB).  During the first
             call all coefficient reads are traced, giving the biquad cascade it evaluates; we
             re-evaluate that cascade in double precision for unclamped magnitude + phase and
             check it against the app's numbers.
  * Filter:  ae::Cchfilter evaluates 4 biquads at chdsp+0x12c; same treatment, checked against
             Cchfilter::getResponseCached on its own 500-point grid.
  * Dyn:     ae::<model>_respf(float in_dB, Cpar** slots) = static output level (dB) the
             DynGraph draws, swept over input level.
"""
import argparse
import hashlib
import itertools
import json
import math
import os
import struct
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curvelib import (Wing, thin_binary, freq_grid, biquad_response, biquad_response_app, G_AE, G_AEDRV,  # noqa: E402
                      AE_CHSTRUCT, EQ_USAGE, EQ_INDEX, EQ_MDL, DYN_PARS, DYN_USAGE, F_TABLE_STRIDE, FS)

HEAP = (0x2_0000_0000, 0x2_4000_0000)
FREQS = freq_grid(10.0, 24000.0, 24)
LEVELS = [round(-90 + 0.5 * i, 1) for i in range(221)]      # -90 .. +20 dB


def r3(v):
    return [round(x, 3) for x in v]


def r2(v):
    return [round(x, 2) for x in v]


class Gen:
    def __init__(self, w):
        self.w = w
        self.e = w.e
        s = w.sym
        self.eq_tab = {0: s["__ZN2aeL10f_table_chE"], 1: s["__ZN2aeL11f_table_busE"],
                       3: s["__ZN2aeL11f_table_auxE"]}
        self.dyn_tab = s["__ZN2aeL7f_tableE"]
        self.drv_lo = self.e.u64(G_AEDRV)

    # ------------------------------------------------------------------ helpers
    def coef_groups(self, reads):
        addrs = []
        for a, sz in reads:
            if sz == 4 and not (self.drv_lo <= a < self.drv_lo + 0x20) and a not in addrs:
                addrs.append(a)
        addrs.sort()
        groups, cur = [], []
        for a in addrs:
            if cur and a != cur[-1] + 4:
                groups.append(cur)
                cur = []
            cur.append(a)
        if cur:
            groups.append(cur)
        bq = []
        for g in groups:
            if len(g) % 5:
                return None
            for i in range(0, len(g), 5):
                bq.append(struct.unpack("<5f", self.e.rd(g[i], 20)))
        return bq

    @staticmethod
    def std_form(bq):
        # memory (b2,b0,b1,A2,A1), y = b0x+b1x1+b2x2 + A1y1 + A2y2  ->  [b0,b1,b2,a1,a2] with a = -A
        return [[b0, b1, b2, -a1 if a1 else 0.0, -a2 if a2 else 0.0] for (b2, b0, b1, a2, a1) in bq]

    def curve_from(self, app_fn, coefs_fn=None, parallel=False):
        """app_fn(f) -> app magnitude; returns dict with mag/phase/biquads/check."""
        w = self.w
        first, reads = w.trace_reads(lambda: app_fn(FREQS[0]), *HEAP)
        app = [first] + [app_fn(f) for f in FREQS[1:]]
        bq = coefs_fn() if coefs_fn else self.coef_groups(reads)
        out = {}
        if bq:
            mag, ph = biquad_response(bq, FREQS, parallel=parallel)
            appf = biquad_response_app(bq, FREQS, parallel=parallel)
            dev = max((abs(a - m) for a, m in zip(app, appf)), default=0.0)
            dev_exact = max((abs(a - m) for a, m in zip(app, mag) if -47.9 < a < 47.9), default=0.0)
            if dev < 0.005:
                out["mag_db"] = r3(mag)
                out["phase_deg"] = r2(ph)
                out["biquads"] = self.std_form(bq)
                if parallel:
                    out["topology"] = "parallel (H = sum of sections)"
                out["check_max_dev_db"] = round(dev, 5)
                out["gui_vs_exact_max_db"] = round(dev_exact, 4)
                return out
            out["note"] = f"traced cascade disagrees with app respf (max {dev:.3f} dB); app magnitude only"
        out["mag_db"] = r3(app)
        out["gui_clamped"] = True
        return out

    def params(self, path, names):
        w = self.w
        return {n: w.value(w.node(f"{path}/{n}")) for n in names}

    def grid(self, path, base, grid, capture, tag=None):
        """Full factorial over grid (ordered dict) after applying base; capture() -> dict."""
        w = self.w
        out = []
        keys = list(grid)
        for combo in itertools.product(*[grid[k] for k in keys]):
            for k, v in base.items():
                w.set(path, k, v)
            for k, v in zip(keys, combo):
                w.set(path, k, v)
            c = capture()
            rec = {"params": self.params(path, list(base) + keys)}
            if tag:
                rec["case"] = tag
            rec.update(c)
            out.append(rec)
        return out

    # ---------------------------------------------------------------------- EQ
    def eq_capture(self, path):
        w, e = self.w, self.e
        n = w.node(path)
        u, idx = e.i32(n + EQ_USAGE), e.i32(n + EQ_INDEX)
        m = w.ival(e.u64(n + EQ_MDL))
        fn = e.u64(self.eq_tab[u] + 0x40 * m + 0x30)
        # eq_respf_maag4 (MACH4) sums its sections (parallel bank) for usage 0, all others cascade
        par = "maag4" in e.addr2sym.get(fn, "") and u == 0
        return lambda: self.curve_from(lambda f: w.call_f(fn, idx, u, fl=(f,)), parallel=par)

    def eq_neutral(self, path, bands):
        return {f"{b}g": 0.0 for b in bands}

    def eq_std(self, path, usage_name, bands, ltypes, cut_types):
        w = self.w
        w.set(path, "mdl", "STD")
        w.refresh(path)
        cap = self.eq_capture(path)
        neutral = self.eq_neutral(path, bands)
        neutral.update({"leq": "SHV", "heq": "SHV"})
        if "tilt" in [nm for nm, _ in w.children(path)]:
            neutral["tilt"] = 0.0
        curves = []
        F = [30, 100, 300, 1000, 3000, 10000, 18000]
        # mid band 2 (all mid bands share code; spot checks below)
        curves += self.grid(path, neutral, {"2g": [-15, -9, -3, 3, 9, 15], "2f": F,
                                            "2q": [0.442, 0.7, 1, 2, 4, 10]}, cap, "mid-peq")
        for b in ("1", "3", "4"):
            curves += self.grid(path, neutral, {f"{b}g": [9], f"{b}f": [1000], f"{b}q": [2]}, cap,
                                "mid-peq-spotcheck")
        for b in ("l", "h"):
            for t in ltypes:
                curves += self.grid(path, dict(neutral, **{f"{b}eq": t}),
                                    {f"{b}g": [-15, -6, 6, 15], f"{b}f": F, f"{b}q": [0.442, 1, 4, 10]},
                                    cap, f"{b}-{t}")
            for t in cut_types:
                qs = [0.442, 0.707, 1, 2, 10] if t == "CUT" else [1]
                curves += self.grid(path, dict(neutral, **{f"{b}eq": t}),
                                    {f"{b}f": F, f"{b}q": qs}, cap, f"{b}-{t}")
        if "tilt" in neutral:
            curves += self.grid(path, neutral, {"tilt": [-6, -3, -1, 1, 3, 6]}, cap, "tilt")
        curves += self.grid(path, neutral, {}, cap, "flat")
        return {"block": path, "usage": usage_name, "model": "STD", "curves": curves}

    # ------------------------------------------------------------------ filter
    def filter_all(self):
        w, e = self.w, self.e
        path = "ch/1/flt"
        w.refresh(path)
        blk = w.node(path)
        ch = e.u64(e.u64(e.u64(G_AE) + AE_CHSTRUCT) + 8 * e.i32(blk + 0xF8))
        gr = e.sym["__ZN2ae9Cchfilter17getResponseCachedEf"]

        def coefs():
            return [struct.unpack("<5f", e.rd(ch + 0x12C + 20 * i, 20)) for i in range(4)]

        def cap():
            bq = coefs()
            mag, ph = biquad_response(bq, FREQS)
            # check against the GUI's own 500-point cache (f_i = 20 * 1.01391^i, float32 iteration)
            e.uc.mem_write(blk + 0x8CC, b"\1")
            f32 = 20.0
            fl, idxs = [], []
            for i in range(500):
                if i % 25 == 0:
                    fl.append(f32)
                    idxs.append(i)
                f32 = struct.unpack("<f", struct.pack("<f", f32 * struct.unpack("<f", struct.pack("<I", 0x3F81C7D9))[0]))[0]
            appv = [w.call_f(gr, blk, fl=(i / 500.0,)) for i in idxs]
            mine = biquad_response_app(bq, fl)
            dev = max((abs(a - m) for a, m in zip(appv, mine)), default=0)
            return {"mag_db": r3(mag), "phase_deg": r2(ph), "biquads": self.std_form(bq),
                    "check_max_dev_db": round(dev, 5)}
        off = {"lc": 0, "hc": 0, "tf": 0}
        curves = []
        curves += self.grid(path, off, {}, cap, "flat")
        for s in ["6", "12", "18", "24"]:
            curves += self.grid(path, dict(off, lc=1, lcs=s),
                                {"lcf": [20, 40, 80, 160, 320, 640, 1280, 2000]}, cap, f"lc-{s}")
        for s in ["6", "12"]:
            curves += self.grid(path, dict(off, hc=1, hcs=s),
                                {"hcf": [50, 200, 500, 1000, 2000, 5000, 10000, 15000, 20000]}, cap, f"hc-{s}")
        curves += self.grid(path, dict(off, lc=1, lcs="24", hc=1, hcs="12"),
                            {"lcf": [100], "hcf": [8000]}, cap, "lc+hc")
        tool = [("TILT", {"tilt": [-6, -3, -1, 1, 3, 6]}),
                ("MAX", {"low": [0, 25, 50, 75, 100], "proc": [0, 25, 50, 75, 100]}),
                ("AP1", {"f": [100, 300, 1000, 3000, 10000]}),
                ("AP2", {"f": [100, 300, 1000, 3000, 10000], "q": [0.5, 1, 2, 5]})]
        for mdl, g in tool:
            w.set(path, "mdl", mdl)
            curves += self.grid(path, dict(off, tf=1), g, cap, f"tool-{mdl}")
        w.set(path, "mdl", "TILT")
        return {"block": path, "curves": curves,
                "note": "Cchfilter = 4 biquads at chdsp+0x12c: LC (up to 2 sections), HC, tool filter."}

    # --------------------------------------------------------- sidechain / xover
    def sidechain(self):
        w, e = self.w, self.e
        path = "ch/1/dynsc"
        w.refresh(path)
        blk = w.node(path)
        fn = e.sym["__ZN2ae13Cdynsidechain11getResponseEf"]
        chs = e.u64(e.u64(e.u64(G_AE) + AE_CHSTRUCT) + 8 * e.i32(blk + 0xD8))
        base = chs + 0xB4 + 0x14 * e.i32(blk + 0xDC)

        def sc_coefs():   # this biquad is stored (A1, A2, b1, b0, b2)
            a1, a2, b1, b0, b2 = struct.unpack("<5f", e.rd(base, 20))
            return [(b2, b0, b1, a2, a1)]
        cap = lambda: self.curve_from(lambda f: w.call_f(fn, blk, fl=(f,)), sc_coefs)  # noqa: E731
        curves = []
        for t in ["OFF", "LP12", "HP12", "BP", "NOTCH"]:
            g = {"f": [50, 200, 1000, 5000, 15000], "q": [0.442, 1, 4, 10] if t in ("BP", "NOTCH") else [1]}
            curves += self.grid(path, {"type": t}, g if t != "OFF" else {}, cap, t)
        w.set(path, "type", "OFF")
        out = {"sidechain": {"block": path, "curves": curves,
                             "note": "ch/*/gatesc uses the same code (usage 0 vs 1 only selects the biquad slot)."}}
        path = "ch/1/dynxo"
        w.refresh(path)
        n = w.node(path)
        fn = e.sym["__ZN2ae10Ccompxover11getResponseEf"]
        chs = e.u64(e.u64(e.u64(G_AE) + AE_CHSTRUCT))

        def capx():
            return self.curve_from(lambda f: w.call_f(fn, 0, 0, fl=(f,)),
                                   lambda: [struct.unpack("<5f", e.rd(chs + 0xDC, 20))])
        curves = []
        for t in ["OFF", "LO", "HI", "S6", "S12", "PEAK"]:
            g = {"f": [50, 200, 1000, 5000, 15000]}
            if t == "PEAK":
                g["q"] = [0.442, 1, 4, 10]
            curves += self.grid(path, {"type": t}, g if t != "OFF" else {}, capx, t)
        w.set(path, "type", "OFF")
        out["dyn_crossover"] = {"block": path, "curves": curves,
                                "note": "Ccompxover::getResponse(int ch, int usage, float f); 1 biquad at chdsp+0xdc. "
                                        "This is the detector/processing band filter of the dyn block (depth not in curve)."}
        _ = n
        return out

    # --------------------------------------------------------------------- dyn
    def dyn_fn(self, path, slot):
        e = self.e
        blk = self.w.node(path)
        u = e.i32(blk + DYN_USAGE)
        m = self.w.ival(e.u64(blk + 0xD8))
        fi = e.i32(self.w.ftblofs + 0x84 * u + 4 * m)
        ent = self.dyn_tab + F_TABLE_STRIDE * fi
        return e.u64(ent + slot), blk, fi

    def dyn_static(self):
        w = self.w
        path = "ch/1/dyn"
        w.refresh(path)
        res = {}

        def mk():
            fn, blk, _ = self.dyn_fn(path, 0x38)
            return lambda: {"out_db": r3([w.call_f(fn, blk + DYN_PARS, fl=(x,)) for x in LEVELS])}
        specs = [
            ("GATE", {"thr": [-80, -60, -40, -20, -10, 0], "range": [3, 10, 20, 40, 60],
                      "ratio": ["1:1.5", "1:2", "1:3", "1:4", "gate"]}),
            ("DUCK", {"thr": [-60, -40, -20, 0], "range": [3, 10, 20, 40, 60]}),
            ("COMP", {"thr": [-60, -40, -20, -10, 0], "ratio": None, "knee": None}),
            ("EXP", {"thr": [-60, -40, -20, -10, 0], "ratio": None, "knee": None}),
        ]
        for mdl, g in specs:
            w.set(path, "mdl", mdl)
            w.refresh(path)
            for k in list(g):
                if g[k] is None:
                    n = w.node(f"{path}/{k}")
                    if w.ftype(n) == 6:
                        g[k] = w.fenum(n)[0]
                    else:
                        lo, hi = self._int_range(n)
                        g[k] = list(range(lo, hi + 1))
            if mdl in ("COMP", "EXP"):
                g = {"thr": g["thr"], "ratio": g["ratio"], "knee": g["knee"]}
            fn, blk, fi = self.dyn_fn(path, 0x38)
            res[mdl] = {"respf": self.e.addr2sym.get(fn), "curves": self.grid(path, {}, g, mk())}
        return {"block": path, "input_db": LEVELS, "models": res}

    def _int_range(self, n):
        import re
        d = self.w.x.describe(n)
        m = re.search(r"\[(-?\d+) \.\. (-?\d+)\]", d)
        return (int(m.group(1)), int(m.group(2))) if m else (0, 1)

    def dyn_inventory(self):
        """Per model: is respf real or a stub, and the GR-meter mapping gain(x, g)."""
        w, e = self.w, self.e
        path = "ch/1/dyn"
        mdl = w.node(f"{path}/mdl")
        names = w.enum_values(mdl)
        grs = [0, 1, 2, 3, 6, 10, 12, 15, 20, 24, 30, 40, 48, 60, 72, 80]
        out = []
        for name in names:
            w.set(path, "mdl", name)
            w.refresh(path)
            fns = {k: self.dyn_fn(path, off)[0] for k, off in
                   (("respf", 0x38), ("led", 0x40), ("state", 0x48), ("gain", 0x50))}
            _, blk, fi = self.dyn_fn(path, 0x38)
            probe = [w.call_f(fns["respf"], blk + DYN_PARS, fl=(x,)) for x in (-60.0, -20.0, 0.0)]
            stub = len(set(probe)) == 1
            meter = [round(w.call_f(fns["gain"], 0, fl=(0.0, 10 ** (-d / 20.0))), 5) for d in grs]
            out.append({"model": name, "f_table_index": fi,
                        "functions": {k: e.addr2sym.get(v) for k, v in fns.items()},
                        "respf_is_stub": stub, "respf_stub_value": probe[0] if stub else None,
                        "gr_meter": {"gain_reduction_db": grs, "meter": meter}})
        w.set(path, "mdl", "GATE")
        return {"block": path, "models": out}

    # ------------------------------------------------------------- alt EQs
    ALT = {
        "SOUL": ({"lg": 0, "lmg": 0, "hmg": 0, "hg": 0}, [
            ("low", {"lg": [-5, -2.5, 2.5, 5], "lf": [0, 2.5, 5, 7.5, 10]}),
            ("lowmid", {"lmg": [-5, 5], "lmf": [0, 5, 10], "lmq": [0, 5, 10], "lmf3": [0, 1]}),
            ("himid", {"hmg": [-5, 5], "hmf": [0, 5, 10], "hmq": [0, 5, 10], "hmf3": [0, 1]}),
            ("high", {"hg": [-5, -2.5, 2.5, 5], "hf": [0, 2.5, 5, 7.5, 10]})]),
        "E88": ({"lg": 0, "lmg": 0, "hmg": 0, "hg": 0}, [
            ("low", {"lg": [-5, 5], "lf": [0, 5, 10], "lq": ["LOW", "HIGH"], "lt": ["BELL", "SHELV"]}),
            ("lowmid", {"lmg": [-5, -2.5, 2.5, 5], "lmf": [0, 5, 10], "lmq": [0, 5, 10]}),
            ("himid", {"hmg": [-5, -2.5, 2.5, 5], "hmf": [0, 5, 10], "hmq": [0, 5, 10]}),
            ("high", {"hg": [-5, 5], "hf": [0, 5, 10], "hq": ["LOW", "HIGH"], "ht": ["BELL", "SHELV"]})]),
        "E84": ({"g": 0, "lg": 0, "mg": 0, "hg": 0}, [
            ("gain", {"g": [-20, -10, 10, 20]}),
            ("low", {"lg": [-5, -2.5, 2.5, 5], "lf": ["35", "60", "110", "220"]}),
            ("mid", {"mg": [-5, 5], "mf": ["350", "700", "1k6", "3k2", "4k8", "7k2"], "mq": ["LOW", "HIGH"]}),
            ("high", {"hg": [-5, -2.5, 2.5, 5], "hf": ["10k", "12k", "16k"]})]),
        "F110": ({"g": 0, "lmg": 0, "hmg": 0, "lg": 0, "hg": 0}, [
            ("gain", {"g": [-18, -9, 9, 18]}),
            ("lowmid", {"lmg": [-5, 5], "lmf": [0, 5, 10], "lmq": [0, 5, 10], "lmf3": [0, 1], "peq": [0, 1]}),
            ("himid", {"hmg": [-5, 5], "hmf": [0, 5, 10], "hmq": [0, 5, 10], "hmf3": [0, 1], "peq": [0, 1]}),
            ("low", {"lg": [-5, 5], "lf": ["33", "56", "95", "160", "270", "460"], "shv": [0, 1]}),
            ("high", {"hg": [-5, 5], "hf": ["3k3", "4k7", "6k8", "10k", "15k", "18k"], "shv": [0, 1]})]),
        "PULSAR": ({"eq1": 1, "eq5": 1, "1lb": 0, "1latt": 0, "1hb": 0, "1hatt": 0, "5lb": 0, "5md": 0,
                    "5hb": 0}, [
            ("p1-low", {"1lb": [0, 5, 10], "1latt": [0, 5, 10], "1lf": ["20", "30", "60", "100"]}),
            ("p1-high", {"1hb": [0, 5, 10], "1hw": [0, 5, 10], "1hf": ["3k", "4k", "5k", "8k", "10k", "12k", "16k"]}),
            ("p1-hatt", {"1hatt": [5, 10], "1hattf": ["5k", "10k", "20k"]}),
            ("m5-low", {"5lb": [5, 10], "5lf": ["200", "300", "500", "700", "1k"]}),
            ("m5-mid", {"5md": [5, 10], "5mf": ["200", "300", "500", "700", "1k", "1k5", "2k", "3k", "4k", "5k", "7k"]}),
            ("m5-high", {"5hb": [5, 10], "5hf": ["1k5", "2k", "3k", "4k", "5k"]})]),
        "MACH4": ({"sub": 0, "40": 0, "160": 0, "650": 0, "2k5": 0, "air": 0}, [
            ("sub", {"sub": [-5, -2.5, 2.5, 5]}), ("40", {"40": [-5, -2.5, 2.5, 5]}),
            ("160", {"160": [-5, -2.5, 2.5, 5]}), ("650", {"650": [-5, -2.5, 2.5, 5]}),
            ("2k5", {"2k5": [-5, -2.5, 2.5, 5]}),
            ("air", {"air": [5, 10], "airm": ["2k5", "5k", "10k", "20k", "40k"]})]),
        "PIA": ({"g": 0, "31": 0, "63": 0, "125": 0, "250": 0, "500": 0, "1k": 0, "2k": 0, "4k": 0, "8k": 0,
                 "16k": 0}, [("gain", {"g": [-12, 12]})] +
                [(b, {b: [-12, -6, 6, 12]}) for b in ["31", "63", "125", "250", "500", "1k", "2k", "4k", "8k", "16k"]]),
    }

    def eq_alt(self, mdl):
        w = self.w
        path = "bus/1/eq" if mdl == "PIA" else "ch/1/eq"
        w.set(path, "mdl", mdl)
        w.refresh(path)
        base, bands = self.ALT[mdl]
        cap = self.eq_capture(path)
        curves = self.grid(path, base, {}, cap, "flat")
        for tag, g in bands:
            curves += self.grid(path, base, g, cap, tag)
        w.set(path, "mdl", "STD")
        return {"block": path, "model": mdl, "curves": curves}


def dump(obj, path, header):
    obj = dict(header, **obj)
    with open(path, "w") as f:
        json.dump(obj, f, separators=(",", ":"))
    print(f"wrote {path} ({os.path.getsize(path) / 1e6:.2f} MB)", file=sys.stderr)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--app", default="WING-Edit.app")
    ap.add_argument("--out", default="data/reference-curves")
    ap.add_argument("--only", default="filter,eq-std,dyn,sc,meter,eq-alt")
    a = ap.parse_args()
    only = set(a.only.split(","))
    exe, thin = thin_binary(a.app)
    sha = hashlib.sha256(open(exe, "rb").read()).hexdigest()
    w = Wing(thin)
    g = Gen(w)
    os.makedirs(a.out, exist_ok=True)
    hdr = {"format": "wing-reference-curves/1",
           "source": {"app": "WING-Edit", "sha256": sha, "slice": "arm64",
                      "generator": "tools/curve-extract/curves.py"},
           "fs": FS,
           "freq_hz": FREQS,
           "conventions": {
               "mag_db": "20*log10|H| of the traced biquad cascade (double precision, unclamped); "
                         "if no cascade could be traced, the app's respf value (float, clamped +-48 dB)",
               "phase_deg": "arg H in degrees, wrapped to (-180,180]",
               "biquads": "[b0,b1,b2,a1,a2], H = (b0+b1 z^-1+b2 z^-2)/(1+a1 z^-1+a2 z^-2), float32 values as computed by WING-Edit",
               "check_max_dev_db": "max |app respf - cascade evaluated with the app's float32 omega/sincosf| (trace verification; curves are kept only if < 0.005 dB)",
               "gui_vs_exact_max_db": "max |app respf - exact double cascade| inside +-47.9 dB (GUI float32 rounding, worst for narrow low-frequency sections)",
               "params": "values read back from the Cpar after ncl::Cpar::set (i.e. quantised to the parameter grid)"}}
    t = time.time()
    if "filter" in only:
        dump(g.filter_all(), os.path.join(a.out, "filter.json"), hdr)
    if "eq-std" in only:
        bus_types = ["PEQ", "SHV"]
        cuts = ["CUT", "BW6", "BW12", "BS12", "LR12", "BW18", "BW24", "BS24", "LR24", "BW48", "LR48"]
        dump(g.eq_std("ch/1/eq", "ch", ["l", "1", "2", "3", "4", "h"], bus_types, []),
             os.path.join(a.out, "eq-std-ch.json"), hdr)
        dump(g.eq_std("aux/1/eq", "aux", ["l", "1", "2", "3", "4", "h"], [], ["CUT"]),
             os.path.join(a.out, "eq-std-aux.json"), hdr)
        dump(g.eq_std("bus/1/eq", "bus/main/mtx", ["l", "1", "2", "3", "4", "5", "6", "h"], bus_types, cuts),
             os.path.join(a.out, "eq-std-bus.json"), hdr)
    if "dyn" in only:
        dump(g.dyn_static(), os.path.join(a.out, "dyn-static.json"), {k: v for k, v in hdr.items() if k != "freq_hz"})
    if "sc" in only:
        dump(g.sidechain(), os.path.join(a.out, "dyn-sidechain-xover.json"), hdr)
    if "meter" in only:
        dump(g.dyn_inventory(), os.path.join(a.out, "dyn-models.json"), {k: v for k, v in hdr.items() if k != "freq_hz"})
    if "eq-alt" in only:
        for m in ["SOUL", "E88", "E84", "F110", "PULSAR", "MACH4", "PIA"]:
            dump(g.eq_alt(m), os.path.join(a.out, f"eq-{m.lower()}.json"), hdr)
    print(f"done in {time.time() - t:.0f}s; unknown imports: "
          f"{ {k: v for k, v in w.e.unknown_calls.items() if 'thread' not in k and 'mutex' not in k} }",
          file=sys.stderr)


if __name__ == "__main__":
    main()
