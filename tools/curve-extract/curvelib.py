"""Shared helpers for driving WING-Edit's own response/gain-curve code in the Unicorn emulator.

The WING-Edit binary is never run natively, modified, or attached to a debugger.  We reuse
tools/schema-extract (wingemu.Emu + extract.Extractor) to build the console's parameter tree
inside the emulator (CaudioEngine::CaudioEngine), then set parameters with the app's own
ncl::Cpar::set()/setInt(), let the owning block recompute its DSP coefficients with its own
update_direct() (which runs the model's *_parchg_* routine), and finally call the GUI's
curve functions (ae::*_respf_*, Cchfilter/Cequalizer response code) or read the biquad
coefficients those functions evaluate.

All addresses/offsets are specific to WING-Edit 3.3.4 (arm64 slice) and are looked up by
symbol where possible; see docs/research/reference-curves.md.
"""
import math
import os
import struct
import subprocess
import sys
import tempfile

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "schema-extract"))
from extract import Extractor, N_FLAGS  # noqa: E402
import wingemu  # noqa: E402
from unicorn import UC_HOOK_MEM_READ  # noqa: E402

# wingemu.Emu (T003) stubs most of libm but not tan/tanf/frexp, which the filter/EQ coefficient
# code (bilinear prewarp) needs.  Wrap its import trap hook instead of editing the T003 tool.
_orig_trap = wingemu.Emu._trap_hook


def _trap_hook(self, uc, addr, size, data):
    name = self.addr_imp.get(addr)
    if name in ("_tan", "_tanf"):
        s = name == "_tanf"
        try:
            self._fret(math.tan(self._farg(0, s)), s)
        except Exception:
            self._fret(float("nan"), s)
        return
    if name == "_frexp":
        m, ex = math.frexp(self._farg(0, False))
        ptr = self.r(wingemu.UC_ARM64_REG_X0)
        self.uc.mem_write(ptr, struct.pack("<i", ex))
        self._fret(m, False)
        return
    return _orig_trap(self, uc, addr, size, data)


wingemu.Emu._trap_hook = _trap_hook

SYM_SET = "__ZN3ncl4Cpar3setEfbi"            # Cpar::set(float value, bool, int)
SYM_SETINT = "__ZN3ncl4Cpar6setIntEibi"      # Cpar::setInt(int, bool, int)
G_AE = 0x1051FD9B0                           # CaudioEngine* _AE
G_AEDRV = 0x1051FB748                        # CdummyAudioEngine* _AEDRV (+0x10 = pi/fs)
AE_CHSTRUCT = 0x6C0                          # _AE->chdsp[ch] (input channels)
AE_BUSSTRUCT = 0x6C8                         # _AE->busdsp[...] (aux/bus/main/mtx)
FS = 48000.0

# Cdynamics
DYN_PARS = 0xE0           # Cpar* slots[] passed to respf
DYN_USAGE = 0x18C         # row of Cdynamics::ftblofs (0x84 bytes per row)
DYN_MDL = 0xD8            # Cpar* mdl
F_TABLE_STRIDE = 0x70     # struct dyn model {name,long,id,pardef,alginit,parchg,init,respf,led,state,gain,...}

# Cequalizer
EQ_USAGE = 0x1B0
EQ_INDEX = 0x1B8
EQ_MDL = 0xB8


def freq_grid(lo=10.0, hi=24000.0, per_oct=24):
    n = int(math.floor(math.log2(hi / lo) * per_oct + 1e-9))
    return [round(lo * 2 ** (i / per_oct), 4) for i in range(n + 1)]


def thin_binary(app):
    exe = os.path.join(app, "Contents/MacOS/WING-Edit")
    tmpd = tempfile.mkdtemp(prefix="wing-curves-")
    thin = os.path.join(tmpd, "we-arm64")
    subprocess.run(["lipo", "-thin", "arm64", exe, "-output", thin], check=True)
    return exe, thin


class Wing:
    def __init__(self, thin):
        self.x = Extractor(thin)
        self.e = self.x.e
        e = self.e
        self.sym = e.sym
        self.f_table = self.sym["__ZN2aeL7f_tableE"] if "__ZN2aeL7f_tableE" in self.sym else None
        self.ftblofs = self.sym["__ZN2ae9Cdynamics7ftblofsE"]
        self._cache = {}

    # ------------------------------------------------------------- tree access
    def node(self, path):
        if path in self._cache:
            return self._cache[path]
        n = self.x.root
        for seg in path.strip("/").split("/"):
            for c in self.x.children(n):
                nm = self.x.name(c)
                if nm == "n/a":
                    nm = str(self.e.i32(c + 0x58))
                if nm == seg:
                    n = c
                    break
            else:
                raise KeyError(f"{path}: no child {seg!r}")
        self._cache[path] = n
        return n

    def children(self, path):
        n = self.node(path)
        return [(self.x.name(c), c) for c in self.x.children(n)]

    def ftype(self, n):
        return (self.e.u32(n + N_FLAGS) >> 20) & 0xF

    def enum_values(self, n):
        cnt = self.e.i32(n + 0xD0)
        a = self.e.u64(n + 0xD8)
        return [self.e.cstr(self.e.u64(a + 8 * i)) for i in range(cnt)]

    def fenum(self, n):
        e = self.e
        cnt = e.i32(n + 0xD0)
        a, la = e.u64(n + 0xD8), e.u64(n + 0xE0)
        vals = [e.f32(a + 4 * i) for i in range(cnt)]
        labs = [e.cstr(e.u64(la + 8 * i)) for i in range(cnt)] if la else [None] * cnt
        return vals, labs

    def value(self, n):
        """Current value as stored: enum token, fenum float, int, or float."""
        t = self.ftype(n)
        if t == 5:
            v = self.enum_values(n)
            i = self.ival(n)
            return v[i] if 0 <= i < len(v) else i
        if t == 6:
            vals, _ = self.fenum(n)
            i = self.ival(n)
            return round(vals[i], 6) if 0 <= i < len(vals) else i
        if t == 4:
            return self.ival(n)
        return round(self.fval(n), 6)

    def fval(self, n):
        return self.e.f32(n + 0xA8)

    def ival(self, n):
        return self.e.i32(n + 0xAC)

    def vmethod(self, blk, key):
        """Address of the virtual method whose mangled name contains key (e.g. '6updateEj')."""
        e = self.e
        vt = e.u64(blk)
        for i in range(48):
            a = e.u64(vt + 8 * i)
            if key in e.addr2sym.get(a, ""):
                return a
        return None

    def refresh(self, blockpath, mask=0xFFFF):
        """Run the block's own update_direct(mask) (re-runs pardef for model changes) and
        update(mask) (computes DSP coefficients via the model's parchg, posts them to the
        CdummyAudioEngine and marks the GUI response cache dirty)."""
        blk = self.node(blockpath)
        for key in ("13update_directEj", "6updateEj"):
            a = self.vmethod(blk, key)
            if a and "ncl5Cnode" not in self.e.addr2sym.get(a, ""):
                self.e.call(a, blk, mask)

    def set(self, blockpath, name, value, update=True):
        """Set blockpath/name to value (float in display units, or enum token / index)."""
        self.node(blockpath)
        self._cache.pop(blockpath + "/" + name, None)
        n = self.node(f"{blockpath}/{name}")
        t = self.ftype(n)
        e = self.e
        if t in (4, 5, 6) or isinstance(value, str):
            if isinstance(value, str):
                vals = self.fenum(n)[1] if t == 6 else self.enum_values(n)
                idx = vals.index(value)
            else:
                idx = int(value)
            if t == 6 and not isinstance(value, str):
                # fenum: value given as float -> nearest entry
                a = e.u64(n + 0xD8)
                cnt = e.i32(n + 0xD0)
                fv = [e.f32(a + 4 * i) for i in range(cnt)]
                idx = min(range(cnt), key=lambda i: abs(fv[i] - float(value)))
            e.call(SYM_SETINT, n, idx, 0, 0)
        else:
            self.call_f(self.sym[SYM_SET], n, 0, 0, fl=(float(value),))
        if update:
            mask = e.u32(n + N_FLAGS) & 0xFFFF
            if mask:
                self.refresh(blockpath, mask)
        if name == "mdl":   # model change renames the slot nodes
            for k in [k for k in self._cache if k.startswith(blockpath + "/")]:
                del self._cache[k]
        return n

    def call_f(self, func, *args, fl=()):
        """Call func(args...) with single-precision float args in s0..; returns s0 as float."""
        from unicorn.arm64_const import UC_ARM64_REG_S0
        dargs = tuple(struct.unpack("<d", struct.pack("<Q", struct.unpack("<I", struct.pack("<f", v))[0]))[0]
                      for v in fl)
        self.e.call(func, *args, fargs=dargs)
        return struct.unpack("<f", struct.pack("<I", self.e.uc.reg_read(UC_ARM64_REG_S0) & 0xFFFFFFFF))[0]

    # ------------------------------------------------------------- read tracing
    def trace_reads(self, fn, lo, hi):
        """Run fn() and return the list of (addr,size) reads that fall in [lo,hi)."""
        reads = []

        def hook(uc, access, addr, size, value, data):
            if lo <= addr < hi:
                reads.append((addr, size))
        h = self.e.uc.hook_add(UC_HOOK_MEM_READ, hook)
        try:
            r = fn()
        finally:
            self.e.uc.hook_del(h)
        return r, reads


def biquad_response(coefs, freqs, fs=FS, parallel=False):
    """coefs: list of (b2, b0, b1, A2, A1) in WING memory order; H = (b0+b1 z^-1+b2 z^-2)/(1-A1 z^-1-A2 z^-2).
    Returns (mag_db, phase_deg) lists."""
    mags, phs = [], []
    for f in freqs:
        w = 2 * math.pi * f / fs
        z1 = complex(math.cos(w), -math.sin(w))
        z2 = z1 * z1
        h = 0j if parallel else 1 + 0j
        for (b2, b0, b1, a2, a1) in coefs:
            hi = (b0 + b1 * z1 + b2 * z2) / (1 - a1 * z1 - a2 * z2)
            h = h + hi if parallel else h * hi
        m = abs(h)
        mags.append(-144.0 if m == 0 else 20 * math.log10(m))
        phs.append(math.degrees(math.atan2(h.imag, h.real)))
    return mags, phs


def _f32(v):
    return struct.unpack("<f", struct.pack("<f", v))[0]


def biquad_response_app(coefs, freqs, fs=FS, parallel=False):
    """Same cascade evaluated the way WING-Edit's respf code does it (omega and sincosf in float32,
    the rest in double, result 10*log10|H|^2 clamped to +-48 dB).  Used only to verify the trace."""
    k = _f32(math.pi / fs)
    out = []
    for f in freqs:
        w = _f32(_f32(f + f) * k)
        s, c = _f32(math.sin(w)), _f32(math.cos(w))
        c2, s2 = 1 - 2 * s * s, -2 * c * s
        p = 1.0
        acc = 0j
        for (b2, b0, b1, a2, a1) in coefs:
            nr = b0 + b1 * c + b2 * c2
            ni = s2 * b2 - b1 * s
            dr = -1 + a1 * c + a2 * c2
            di = s2 * a2 - a1 * s
            if parallel:
                acc += complex(nr, ni) / complex(dr, di)
            else:
                p *= (nr * nr + ni * ni) / (dr * dr + di * di)
        if parallel:
            p = _f32(abs(acc) ** 2)
        db = -144.0 if p == 0 else _f32(math.log(p) * 4.342944819032518)
        out.append(max(-48.0, min(48.0, db)))
    return out
