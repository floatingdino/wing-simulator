"""FX-rack EQ/dyn oracle (T05Q): run WING-Edit's own `ae::fx_parchg_<model>` in the emulator and record the
coefficient block it posts to the audio engine (CdummyAudioEngine vtable+8), for

* GEQ (`fx_parchg_geq`, 31 bands + TRIM, STD and TRU),
* the FX-rack EQs SOUL/E88/E84/F110/PULSAR/MACH4 and PIA (`fx_parchg_{ssl_g,neve88rs,neve84,isa110,
  pultec_duo,maag4,api560}`), slot 1 = `mix`,
* SPKMAN's linear part (`fx_parchg_spkpro`: HP/LP crossover, tilt shelf, phase all-pass + invert, distance /
  position delays).

The binary is only emulated function by function (Unicorn, T003 harness); it is never run natively.
`_AEDRV+0xc` (sample period, 0 in WING-Edit's dummy engine) is set to 1/fs first.

usage: VENV/bin/python -I fx_eqdyn.py <thin arm64 WING-Edit> <out.json> [n_per_model] [seed]
       (thin: lipo -thin arm64 WING-Edit.app/Contents/MacOS/WING-Edit -output we-arm64)

Output (`data/reference-curves/fx-eqdyn.json`): {format, source, fs, geq: [...], eq: [...], spkman: [...]};
every case has `values` = the model's slot values in schema slot order (slot k = values[k-1]) read back after
`Cpar::set` (enums as index, fenums as value), and `biquads` = [[b0,b1,b2,a1,a2], ...] (a = -A of WING's
memory order). SPKMAN cases also have `delay` = [L, R] in samples."""
import hashlib
import json
import os
import random
import struct
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from curvelib import Wing, G_AEDRV, N_FLAGS  # noqa: E402,F401
from unicorn import UC_HOOK_CODE  # noqa: E402
from unicorn.arm64_const import UC_ARM64_REG_X1, UC_ARM64_REG_X2, UC_ARM64_REG_X4  # noqa: E402

FS = 48000.0
FIXED = ("mdl", "fxmix", "$esrc", "$emode", "$a_chn", "$a_pos")


class Fx:
    def __init__(self, thin, slot=1):
        self.w = Wing(thin)
        self.e = e = self.w.e
        self.path = f"fx/{slot}"
        drv = e.u64(G_AEDRV)
        e.uc.mem_write(drv + 0xC, struct.pack("<f", 1.0 / FS))
        post = e.u64(e.u64(drv) + 8)
        self.posts = []

        def hook(uc, addr, size, data):
            r = uc.reg_read
            n = r(UC_ARM64_REG_X2) & 0xFFFFFFFF
            raw = e.rd(r(UC_ARM64_REG_X4), 4 * n) if 0 < n < 4096 else b""
            self.posts.append((r(UC_ARM64_REG_X1) & 0xFFFFFFFF, list(struct.unpack("<%df" % n, raw)),
                               list(struct.unpack("<%dI" % n, raw))))
        e.uc.hook_add(UC_HOOK_CODE, hook, begin=post, end=post)
        e.uc.ctl_flush_tb()

    def model(self, m):
        self.w.set(self.path, "mdl", m)
        self.w.refresh(self.path)
        self.names = [n for n, _ in self.w.children(self.path) if n not in FIXED and n != "n/a"]

    def set(self, k, v):
        self.w.set(self.path, k, v, update=False)

    def values(self):
        out = []
        for n in self.names:
            node = self.w.node(f"{self.path}/{n}")
            t = self.w.ftype(node)
            if t in (4, 5):
                out.append(self.w.ival(node))
            elif t == 6:
                out.append(round(self.w.fenum(node)[0][self.w.ival(node)], 6))
            else:
                out.append(self.w.fval(node))
        return out

    def run(self):
        """update(0xffff) only: update_direct would re-init the model's parameters."""
        self.posts.clear()
        blk = self.w.node(self.path)
        self.e.call(self.w.vmethod(blk, "6updateEj"), blk, 0xFFFF)
        return list(self.posts)


def bqs(fl, stereo=False):
    if stereo:
        fl = fl[::2]
    return [[f[1], f[2], f[0], -f[4], -f[3]] for f in (fl[i:i + 5] for i in range(0, len(fl) - 4, 5))]


def cat(posts, lo, hi):
    """Concatenate posted floats with DSP word addresses in [lo, hi) (contiguous posts)."""
    out = []
    for a, fl, _ in sorted(posts, key=lambda p: p[0]):
        if lo <= a < hi:
            out += fl
    return out


def knob(lo, hi, step=None):
    r = random.random()
    v = lo if r < 0.1 else hi if r < 0.2 else 0.0 if (r < 0.3 and lo < 0 < hi) else random.uniform(lo, hi)
    return round(v / step) * step if step else round(v, 3)


EQ_JOBS = {
    "SOUL": {"lf": (0, 10), "lg": (-5, 5), "lmf": (0, 10), "lmf3": 2, "lmq": (0, 10), "lmg": (-5, 5),
             "hmf": (0, 10), "hmf3": 2, "hmq": (0, 10), "hmg": (-5, 5), "hf": (0, 10), "hg": (-5, 5)},
    "E88": {"lf": (0, 10), "lg": (-5, 5), "lq": 2, "lt": 2, "lmf": (0, 10), "lmg": (-5, 5), "lmq": (0, 10),
            "hmf": (0, 10), "hmg": (-5, 5), "hmq": (0, 10), "hf": (0, 10), "hg": (-5, 5), "hq": 2, "ht": 2},
    "E84": {"g": (-20, 20), "lf": 5, "lg": (-5, 5), "mf": 7, "mg": (-5, 5), "mq": 2, "hf": 4, "hg": (-5, 5)},
    "F110": {"peq": 2, "lmf": (0, 10), "lmg": (-5, 5), "lmq": (0, 10), "lmf3": 2, "hmf": (0, 10), "hmg": (-5, 5),
             "hmq": (0, 10), "hmf3": 2, "shv": 2, "lf": 6, "lg": (-5, 5), "hf": 6, "hg": (-5, 5), "g": (-18, 18)},
    "PULSAR": {"eq1": 2, "1lb": (0, 10), "1latt": (0, 10), "1lf": 4, "1hw": (0, 10), "1hb": (0, 10), "1hf": 7,
               "1hatt": (0, 10), "1hattf": 3, "eq5": 2, "5lb": (0, 10), "5lf": 5, "5md": (0, 10), "5mf": 11,
               "5hb": (0, 10), "5hf": 5},
    "MACH4": {"sub": (-5, 5), "40": (-5, 5), "160": (-5, 5), "650": (-5, 5), "2k5": (-5, 5), "air": (0, 10),
              "airm": 6, "again": 2},
    "PIA": dict({"g": (-12, 12)}, **{b: (-12, 12) for b in
                                     ["31", "63", "125", "250", "500", "1k", "2k", "4k", "8k", "16k"]}),
}
SPK_TYPES = 11


def main():
    thin, out = sys.argv[1], sys.argv[2]
    n = int(sys.argv[3]) if len(sys.argv) > 3 else 12
    random.seed(int(sys.argv[4]) if len(sys.argv) > 4 else 5)
    fx = Fx(thin)
    res = {"format": "wing-fx-coefficients/1",
           "source": {"app": "WING-Edit", "slice": "arm64",
                      "sha256": hashlib.sha256(open(thin, "rb").read()).hexdigest(),
                      "generator": "tools/curve-extract/fx_eqdyn.py"},
           "fs": FS, "geq": [], "eq": [], "spkman": []}

    # ---- GEQ
    fx.model("GEQ")
    bands = fx.names[1:32]
    for ty in (0, 1):
        for k in range(n + 4):
            fx.set("type", ty)
            for i, b in enumerate(bands):
                if k < 4:   # single band / neighbours
                    v = 12.0 if i in ([5, 17, 30, 0][k],) else (-6.0 if k == 1 and i == 18 else 0.0)
                else:
                    v = knob(-15, 15, 0.25)
                fx.set(b, v)
            fx.set("TRIM", 0.0 if k < 4 else knob(-15, 15, 0.25))
            posts = fx.run()
            first = min(p[0] for p in posts if len(p[1]) == 64)
            coefs = cat(posts, first, first + 155)
            res["geq"].append({"values": fx.values(), "biquads": bqs(coefs)})

    # ---- FX EQs (slot 1 = mix)
    for m, spec in EQ_JOBS.items():
        fx.model(m)
        for k in range(n):
            for key, s in spec.items():
                if isinstance(s, tuple):
                    fx.set(key, knob(*s))
                else:
                    fx.set(key, random.randrange(s))
            if fx.w.node(f"{fx.path}/mix") and m not in ("PULSAR", "MACH4"):
                fx.set("mix", 100.0 if k < n // 2 else round(random.uniform(0, 125)))
            posts = fx.run()
            big = max(posts, key=lambda p: len(p[1]))
            res["eq"].append({"model": m, "values": fx.values(), "biquads": bqs(big[1])})

    # ---- SPKMAN (linear part)
    fx.model("SPKMAN")
    for k in range(3 * n):
        fx.set("hpf", knob(20, 2000))
        fx.set("hptype", random.randrange(SPK_TYPES))
        fx.set("lpf", knob(200, 20000))
        fx.set("lptype", random.randrange(SPK_TYPES))
        fx.set("tiltf", knob(100, 10000))
        fx.set("tiltg", knob(-6, 6))
        fx.set("phase", 0.0 if k % 4 == 0 else knob(0, 180, 5))
        fx.set("invert", random.randrange(2))
        fx.set("dist", knob(0, 5, 0.01))
        fx.set("pos", knob(-5, 5, 0.01))
        posts = fx.run()
        first = min(p[0] for p in posts if len(p[1]) == 64)
        coefs = cat(posts, first, first + 100)
        # delays: the one 2-word post of small integers (samples * 2, L/R)
        delay = [w for a, fl, w in posts if len(w) == 2 and max(w) <= 8000 and w != [0, 0]]
        res["spkman"].append({"values": fx.values()[:10], "biquads": bqs(coefs, stereo=True),
                              "delay": [x // 2 for x in delay[-1]] if delay else [0, 0]})
    with open(out, "w") as f:
        json.dump(res, f, separators=(",", ":"))


if __name__ == "__main__":
    main()
