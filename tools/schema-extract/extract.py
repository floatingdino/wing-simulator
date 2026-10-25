#!/usr/bin/env python3
"""Extract the WING parameter tree from WING-Edit.app into data/schema/wing-tree.json.

Method (see docs/research/wing-edit-schema.md):
  * WING-Edit builds the console's whole parameter tree in memory (namespace
    `ncl`, nodes = ncl::Cnode, parameters = ncl::Cpar subclasses) inside
    CaudioEngine::CaudioEngine().  We run *only that constructor* (and a few
    tree helpers) inside the Unicorn CPU emulator against the arm64 slice of
    the unmodified binary; every OS/library import is a Python stub.
  * Each parameter is then described with the app's own
    ncl::Cpar::describe(Ctextstream*) routine (the same text the console
    returns for a "describe" request) and its raw definition fields are read.
  * Model-dependent parameter slots (filter/gate/EQ/dynamics/FX models, ...)
    are enumerated by setting every value of every enum parameter on one
    instance of each block and calling the block's own update_direct(), which
    re-runs the model's pardef function, then diffing the subtree.

Usage:
  python3 -m venv VENV && VENV/bin/pip install unicorn capstone lief
  VENV/bin/python -I tools/schema-extract/extract.py \
      --app WING-Edit.app --snap fixtures/PUBLIC.snap --out data/schema/wing-tree.json \
      [--dump-default-snaps DIR] [--coverage-report FILE]
"""
import argparse
import hashlib
import json
import os
import re
import struct
import sys
import tempfile
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from wingemu import Emu  # noqa: E402
from winghash import wing_hash  # noqa: E402

# ---- binary-specific constants (WING-Edit 3.3.4, arm64 slice) ----------------
SYM_AE_CTOR = "__ZN12CaudioEngineC2Ev"
SYM_SETROOT = "__ZN3ncl5Cnode7setRootEPNS_5CrootE"
SYM_GETPATH = "__ZN3ncl5Cnode7getPathEP11Ctextstreamb"
SYM_DESCRIBE = "__ZN3ncl4Cpar8describeEP11Ctextstreamb"
SYM_SETINT = "__ZN3ncl4Cpar6setIntEibi"
SYM_CSTR32K_VT = "__ZTV4CstrILi32768EE"           # Ctextstream backed by a buffer
SYM_DUMMY_AE_VT = "__ZTV17CdummyAudioEngine"
G_DSPIF_1 = 0x1051e48b0      # globals set by MainComponent before CaudioEngine()
G_DSPIF_2 = 0x1051fb748
G_AE = 0x1051fd9b0
UNIT_TABLE = 0x105097458      # const char* units[7]

# Cnode / Cpar field offsets (derived from disassembly, see doc)
N_NAME, N_LONG, N_IDX, N_FLAGS, N_ROOT, N_CHILD, N_NEXT, N_HASH = \
    0x48, 0x50, 0x58, 0x5C, 0x60, 0x70, 0x78, 0x88
P_VALF, P_MIN, P_STEP, P_NSTEPS, P_ENUM, P_ENUMLONG, P_FMT = \
    0xA8, 0xC8, 0xCC, 0xD0, 0xD8, 0xE0, 0xE8

TYPE_NIBBLE = {0x1: "linf", 0x2: "logf", 0x3: "fader", 0x4: "int", 0x5: "enum",
               0x6: "fenum", 0x7: "string"}


class Extractor:
    def __init__(self, macho):
        self.e = Emu(macho)
        e = self.e
        units = []
        for i in range(7):
            units.append(e.cstr(e.u64(UNIT_TABLE + 8 * i)).strip())
        self.units = units
        # CdummyAudioEngine instance (the DSP interface WING-Edit uses offline)
        dum = e.malloc(0x20)
        e.uc.mem_write(dum, struct.pack("<Q", e.sym[SYM_DUMMY_AE_VT] + 0x10))
        e.uc.mem_write(dum + 8, e.rd(0x1005F3B60, 8))
        e.uc.mem_write(dum + 0x10, struct.pack("<I", 0x3889421E))
        e.uc.mem_write(G_DSPIF_1, struct.pack("<Q", dum))
        e.uc.mem_write(G_DSPIF_2, struct.pack("<Q", dum))
        self.root = e.malloc(0xCA0)
        t = time.time()
        e.call(SYM_AE_CTOR, self.root)
        e.uc.mem_write(G_AE, struct.pack("<Q", self.root))
        # node hashing exactly like MainComponent: count, allocate, fill
        e.call(SYM_SETROOT, self.root, self.root)
        cnt = e.i32(self.root + 0xB8)
        e.uc.mem_write(self.root + 0xC0, struct.pack("<QQ", e.malloc(4 * cnt), e.malloc(8 * cnt)))
        e.uc.mem_write(self.root + 0xB8, struct.pack("<i", 0))
        e.call(SYM_SETROOT, self.root, self.root)
        self.node_count = cnt
        print(f"tree built in emulator: {cnt} nodes ({time.time() - t:.1f}s)", file=sys.stderr)
        self.stream = self._mkstream()
        self.vt_cache = {}

    # ---------------------------------------------------------- primitives
    def _mkstream(self, cap=1 << 16):
        e = self.e
        s = e.malloc(0x40)
        buf = e.malloc(cap + 8)
        e.uc.mem_write(s, struct.pack("<QiiiiQ", e.sym[SYM_CSTR32K_VT] + 0x10, 0, 0, cap, 0, buf))
        return s

    def _sread(self):
        e = self.e
        n = e.i32(self.stream + 0xC)
        return e.rd(e.u64(self.stream + 0x18), n).decode("latin-1")

    def _sreset(self):
        self.e.uc.mem_write(self.stream + 8, struct.pack("<ii", 0, 0))

    def children(self, n):
        e = self.e
        c = e.u64(n + N_CHILD)
        first = c
        out = []
        while c:
            out.append(c)
            c = e.u64(c + N_NEXT)
            if c == first:
                break
        return out

    def is_par(self, n):
        return (self.e.u32(n + N_FLAGS) >> 16) & 0xF0 != 0

    def name(self, n):
        nm = self.e.cstr(self.e.u64(n + N_NAME))
        return nm if nm is not None else str(self.e.i32(n + N_IDX))

    def getpath(self, n):
        self._sreset()
        self.e.call(SYM_GETPATH, n, self.stream, 0)
        return self._sread()

    def describe(self, n):
        self._sreset()
        self.e.call(SYM_DESCRIBE, n, self.stream, 1)
        return self._sread().rstrip("\n")

    def cls(self, n):
        vt = self.e.u64(n)
        if vt not in self.vt_cache:
            s = self.e.addr2sym.get(vt - 0x10, "?")
            m = re.match(r"__ZTVN(\d+)(\w+)E$", s)
            if m:   # __ZTVN3ncl7CparIntE -> CparInt, __ZTVN2ae7CparTagE -> ae::CparTag
                rest, parts = m.group(1) + m.group(2), []
                while rest and rest[0].isdigit():
                    k = re.match(r"\d+", rest).group(0)
                    parts.append(rest[len(k):len(k) + int(k)])
                    rest = rest[len(k) + int(k):]
                name = parts[-1] if parts and parts[0] == "ncl" else "::".join(parts)
            else:
                name = s
            self.vt_cache[vt] = name
        return self.vt_cache[vt]

    def strarr(self, a, n):
        e = self.e
        if not a:
            return None
        return [e.cstr(e.u64(a + 8 * i)) for i in range(n)]

    # ------------------------------------------------------------ params
    def par_def(self, n):
        """Machine-readable definition of one ncl::Cpar."""
        e = self.e
        flags = e.u32(n + N_FLAGS)
        tnib = (flags >> 20) & 0xF
        unib = (flags >> 16) & 0xF
        d = {
            "type": TYPE_NIBBLE.get(tnib, f"t{tnib}"),
            "class": self.cls(n),
        }
        if unib:
            d["unit"] = self.units[unib - 1] if unib <= len(self.units) else f"u{unib}"
        desc = self.describe(n)
        d["_cur"] = desc[:20].strip()
        t = d["type"]
        if t not in ("enum", "string"):
            d["describe"] = desc[20:].strip() if len(desc) > 20 else desc.strip()
        mm = re.search(r"\[(.*) \.\. (.*?)\]", desc)
        if t in ("linf", "logf"):
            mn = e.f32(n + P_MIN)
            st = e.f32(n + P_STEP)
            ns = e.i32(n + P_NSTEPS)
            d["min"] = round(mn, 6)
            d["steps"] = ns
            if t == "linf":
                d["step"] = round(st, 7)
                d["max"] = round(mn + st * (ns - 1), 5)
            else:
                import math
                d["logstep"] = st
                d["max"] = round(mn * math.exp(st * (ns - 1)), 4)
            fmt = e.cstr(e.u64(n + P_FMT))
            if fmt:
                d["fmt"] = fmt
        elif t == "fader":
            d["min"] = -144.0
            d["max"] = 10.0
            d["steps"] = e.i32(n + P_NSTEPS)
        elif t == "int":
            if mm:
                try:
                    d["min"], d["max"] = int(mm.group(1)), int(mm.group(2))
                except ValueError:
                    pass
        elif t == "enum":
            cnt = e.i32(n + P_NSTEPS)
            d["values"] = self.strarr(e.u64(n + P_ENUM), cnt)
            ln = self.strarr(e.u64(n + P_ENUMLONG), cnt)
            if ln and ln != d["values"]:
                d["long_values"] = ln
        elif t == "fenum":
            cnt = e.i32(n + P_NSTEPS)
            a = e.u64(n + P_ENUM)
            d["values"] = [e.f32(a + 4 * i) for i in range(cnt)] if a else None
            d["labels"] = self.strarr(e.u64(n + P_ENUMLONG), cnt)
        elif t == "string":
            d["maxlen"] = e.i32(n + P_NSTEPS)
        return d

    # --------------------------------------------------------------- walk
    def walk(self, n, path=""):
        """Return a full (untemplated) dict for node n and its subtree."""
        e = self.e
        nm = self.name(n)
        na = nm == "n/a"          # unused model slot: getPath() prints its index instead
        if na:
            nm = str(self.e.i32(n + N_IDX))
        p = f"{path}/{nm}" if path else nm
        d = {"name": nm, "hash": f"{e.u32(n + N_HASH):08x}",
             "flags": f"{e.u32(n + N_FLAGS):08x}"}
        ln = e.cstr(e.u64(n + N_LONG))
        if ln is not None:
            d["long"] = ln
        if e.cstr(e.u64(n + N_NAME)) is None:
            d["indexed"] = True
        if na:
            d["unused_slot"] = True
        if self.is_par(n):
            d.update(self.par_def(n))
            d["kind"] = "par"
        else:
            d["kind"] = "node"
            d["children"] = [self.walk(c, p) for c in self.children(n)]
        d["_addr"] = n
        d["_path"] = p
        d["_idx"] = e.i32(n + N_IDX)
        return d


# ----------------------------------------------------------------- template
def sig(d, top=True):
    """Structural signature ignoring per-instance details (hash, long name of indexed nodes)."""
    keys = ("name", "kind", "type", "class", "unit", "min", "max", "steps", "values", "maxlen")
    s = [d.get(k) for k in keys]
    if top and d.get("indexed"):
        s[0] = "#"          # instance number / long name may differ
    else:
        s.append(d.get("long"))
    s.append(d.get("describe", ""))
    s.append(tuple(sig(c, False) for c in d.get("children", [])))
    return repr(s)


def templatize(d):
    """Collapse runs of indexed siblings with identical structure into one template."""
    out = {k: v for k, v in d.items() if not k.startswith("_") and k != "children"}
    if d["kind"] != "node":
        return out
    kids = d["children"]
    res = []
    i = 0
    while i < len(kids):
        k = kids[i]
        if k.get("indexed"):
            j = i + 1
            sk = sig(k)
            while j < len(kids) and kids[j].get("indexed") and sig(kids[j]) == sk and \
                    int(kids[j]["name"]) == int(kids[j - 1]["name"]) + 1:
                j += 1
            if j - i > 1:
                t = templatize(k)
                t.pop("hash", None)
                t.pop("long", None)
                t["name"] = "*"
                grp = {"kind": "array", "first": int(k["name"]), "last": int(kids[j - 1]["name"]),
                       "longs": [x.get("long") for x in kids[i:j]],
                       "template": t}
                if all(x is None for x in grp["longs"]):
                    del grp["longs"]
                res.append(grp)
                i = j
                continue
        res.append(templatize(k))
        i += 1
    out["children"] = res
    return out


def strip_hash_in_template(t):
    t.pop("hash", None)
    for c in t.get("children", []):
        strip_hash_in_template(c if c.get("kind") != "array" else c["template"])


# ------------------------------------------------------------ variant probing
def flat_params(d, rel=""):
    """[(relpath, node-dict)] for all params below d (excluding d)."""
    out = []
    for c in d.get("children", []):
        p = f"{rel}/{c['name']}" if rel else c["name"]
        if c["kind"] == "par":
            out.append((p, c))
        else:
            out.extend(flat_params(c, p))
    return out


def probe_variants(x, full, log):
    """For one representative instance of every distinct block, iterate every enum
    parameter value, let the block redefine itself (update_direct) and record which
    sibling parameter definitions change.  Returns {block_pattern: {selector: {...}}}."""
    e = x.e
    reps = []

    def collect(d, pattern):
        # representative = first instance of each structurally distinct indexed run
        if d["kind"] == "par":
            return
        kids = d["children"]
        seen = set()
        for c in kids:
            if c.get("indexed"):
                s = sig(c)
                if s in seen:
                    continue
                seen.add(s)
                cp = f"{pattern}/{c['name']}" if pattern else c["name"]
            else:
                cp = f"{pattern}/{c['name']}" if pattern else c["name"]
            if c["kind"] == "node":
                if any(k["kind"] == "par" and k.get("type") == "enum" for k in c["children"]):
                    reps.append((cp, c))
                collect(c, cp)

    collect(full, "")
    results = {}
    nprobe = 0
    for bp, block in reps:
        addr = block["_addr"]
        vt = e.u64(addr)
        ud = None
        for i in range(48):
            a = e.u64(vt + 8 * i)
            if "update_direct" in e.addr2sym.get(a, ""):
                ud = a
                break
        if ud is None or e.addr2sym.get(ud, "") == "__ZN3ncl5Cnode13update_directEj":
            continue
        enums = [c for c in block["children"] if c["kind"] == "par" and c.get("type") == "enum"]

        def snapshot():
            return {c["_addr"]: e.rd(c["_addr"] + 0x48, 0xC0) for c in walk_nodes(block)}

        for ep in enums:
            pa = ep["_addr"]
            mask = e.u32(pa + N_FLAGS) & 0xFFFF
            if not mask:
                continue
            orig = e.i32(pa + 0xAC)
            vals = ep.get("values") or []
            base = None
            variants = {}
            changed_any = False
            for vi, vn in enumerate(vals):
                try:
                    e.call(SYM_SETINT, pa, vi, 0, 0)
                    e.call(ud, addr, mask)
                except Exception as ex:  # noqa: BLE001
                    log.append(f"probe {bp}.{ep['name']}={vn}: {ex}")
                    continue
                nprobe += 1
                snap = snapshot()
                if base is None:
                    base = snap
                elif snap != base:
                    changed_any = True
                sub = x.walk(addr, "")
                variants[vn] = sub
            try:
                e.call(SYM_SETINT, pa, max(orig, 0), 0, 0)
                e.call(ud, addr, mask)
            except Exception:
                pass
            if not changed_any:
                continue
            # keep only parameters that are not identical across all variants
            allp = {vn: dict(flat_params(sub)) for vn, sub in variants.items()}
            keys = set()
            for m in allp.values():
                keys |= set(m)
            common = set()
            for k in keys:
                defs = [json.dumps({kk: vv for kk, vv in clean_par(m.get(k)).items() if kk != "hash"},
                                   sort_keys=True) if m.get(k) else None
                        for m in allp.values()]
                if len(set(defs)) == 1:
                    common.add(k)
            if keys <= common:
                continue
            out = []
            for vi, (vn, m) in enumerate(allp.items()):
                ent = {"name": vn, "index": vi}
                if ep.get("long_values"):
                    ent["long"] = ep["long_values"][vi]
                plist = []
                for k, pd in m.items():
                    if k in common or k == ep["name"]:
                        continue
                    if pd.get("unused_slot"):
                        continue
                    q = clean_par(pd)
                    q["default"] = pd.get("_cur")
                    if pd.get("_idx"):
                        q["slot"] = pd["_idx"]   # hash = wing_hash(<block path>/<slot>)
                    q["path"] = k
                    plist.append(q)
                ent["params"] = plist
                out.append(ent)
            results.setdefault(bp, {})[ep["name"]] = {
                "selector": f"{bp}/{ep['name']}",
                "representative": bp,
                "max_params": max(len(v["params"]) for v in out),
                "variants": out,
            }
            log.append(f"variants: {bp}/{ep['name']} -> {len(out)} values")
    print(f"variant probes: {nprobe}", file=sys.stderr)
    return results


def walk_nodes(d):
    yield d
    for c in d.get("children", []):
        yield from walk_nodes(c)


def clean_par(p):
    if p is None:
        return None
    return {k: v for k, v in p.items() if not k.startswith("_")}


# ----------------------------------------------------------------- coverage
def snap_leaves(snap):
    out = []

    def rec(o, p):
        if isinstance(o, dict):
            for k, v in o.items():
                rec(v, f"{p}/{k}" if p else k)
        else:
            out.append((p, o))
    for sect in ("ae_data", "ce_data", "ae_globals", "ce_globals"):
        if sect in snap:
            rec(snap[sect], sect)
    return out


SECTION_ROOT = {"ae_data": "", "ce_data": "$ctl", "ae_globals": "$globals",
                "ce_globals": "$ctl/$globals"}


def coverage(snap, full, variants):
    """Fraction of snapshot leaf keys that resolve to a parameter in the tree, where
    model-dependent keys are resolved via the snapshot's own selector values."""
    idx = {}

    def rec(d, p):
        idx[p] = d
        for c in d.get("children", []):
            rec(c, f"{p}/{c['name']}" if p else c["name"])
    rec(full, "")
    # pattern -> variant tables
    leaves = snap_leaves(snap)
    snapvals = {}
    for p, v in leaves:
        sect, _, rest = p.partition("/")
        tp = f"{SECTION_ROOT[sect]}/{rest}".strip("/") if SECTION_ROOT[sect] else rest
        snapvals[tp] = v
    hit, miss, via_variant = 0, [], 0
    for tp, v in snapvals.items():
        d = idx.get(tp)
        if d is not None and d["kind"] == "par" and not d.get("unused_slot"):
            hit += 1
            continue
        # try model variants: find block whose pattern matches the parent path
        parent, _, leaf = tp.rpartition("/")
        ok = False
        for bp, sels in variants.items():
            if not re.fullmatch(pattern_regex(bp), parent):
                continue
            for selname, sel in sels.items():
                sv = snapvals.get(f"{parent}/{selname}")
                for var in sel["variants"]:
                    if str(var["name"]) == str(sv) and any(q["path"] == leaf for q in var["params"]):
                        ok = True
        if ok:
            hit += 1
            via_variant += 1
        else:
            miss.append(tp)
    return {"total": len(snapvals), "covered": hit, "via_model_variants": via_variant,
            "percent": round(100.0 * hit / max(len(snapvals), 1), 2), "missing": miss}


def pattern_regex(bp):
    # representative paths use the first instance index; generalise numbers
    return "/".join(r"\d+" if seg.isdigit() else re.escape(seg) for seg in bp.split("/"))


# --------------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--app", default="WING-Edit.app")
    ap.add_argument("--snap", default="fixtures/PUBLIC.snap")
    ap.add_argument("--out", default="data/schema/wing-tree.json")
    ap.add_argument("--dump-default-snaps", default=None)
    ap.add_argument("--extra-snap", action="append", default=[],
                    help="additional snapshot(s) to compute coverage against")
    ap.add_argument("--coverage-report", default=None)
    a = ap.parse_args()

    exe = os.path.join(a.app, "Contents/MacOS/WING-Edit")
    raw = open(exe, "rb").read()
    sha = hashlib.sha256(raw).hexdigest()
    tmpd = tempfile.mkdtemp(prefix="wing-schema-")
    thin = os.path.join(tmpd, "we-arm64")
    import subprocess
    subprocess.run(["lipo", "-thin", "arm64", exe, "-output", thin], check=True)
    info = subprocess.run(["plutil", "-extract", "CFBundleShortVersionString", "raw",
                           os.path.join(a.app, "Contents/Info.plist")],
                          capture_output=True, text=True).stdout.strip()

    x = Extractor(thin)
    full = x.walk(x.root, "")
    full["name"] = ""

    # hash algorithm self-check over every node
    bad = 0
    total = 0
    for d in walk_nodes(full):
        if d is full:
            continue
        total += 1
        p = x.getpath(d["_addr"])
        if f"{wing_hash(p):08x}" != d["hash"]:
            bad += 1
    print(f"hash check: {total - bad}/{total} match", file=sys.stderr)

    log = []
    variants = probe_variants(x, full, log)

    # embedded default snapshots
    defaults = {}
    for m in re.finditer(rb'\{"type":"snapshot\.\d+"', raw):
        end = raw.index(b"\0", m.start())
        try:
            sj = json.loads(raw[m.start():end])
        except Exception:
            continue
        defaults.setdefault(sj.get("creator_model"), sj)
    if a.dump_default_snaps:
        os.makedirs(a.dump_default_snaps, exist_ok=True)
        for k, v in defaults.items():
            with open(os.path.join(a.dump_default_snaps, f"default-{k}.snap"), "w") as f:
                json.dump(v, f, indent=1)

    cov = {}
    if os.path.exists(a.snap):
        cov["PUBLIC.snap"] = coverage(json.load(open(a.snap)), full, variants)
    for k, v in defaults.items():
        cov[f"embedded default {k}"] = coverage(v, full, variants)
    for p in a.extra_snap:
        cov[os.path.relpath(p)] = coverage(json.load(open(p)), full, variants)

    # attach defaults from the embedded wing-fullsize default snapshot (for arrays the
    # template carries instance 1's default)
    dsnap = defaults.get("wing-fullsize")
    if dsnap:
        dv = {}
        for p, v in snap_leaves(dsnap):
            sect, _, rest = p.partition("/")
            dv[f"{SECTION_ROOT[sect]}/{rest}".strip("/") if SECTION_ROOT[sect] else rest] = v

        def put(d, p):
            if d["kind"] == "par" and p in dv:
                d["default"] = dv[p]
            for c in d.get("children", []):
                put(c, f"{p}/{c['name']}" if p else c["name"])
        put(full, "")

    hashes = {}

    def hrec(d, p):
        for c in d.get("children", []):
            cp = f"{p}/{c['name']}" if p else c["name"]
            hashes[cp] = c["hash"]
            hrec(c, cp)
    hrec(full, "")

    tree = templatize(full)
    tree["name"] = "/"
    strip_hash_in_arrays(tree)

    # key by generalised pattern (ch/*/dyn); when structurally different instance runs
    # share a pattern (fx 1-8 "full" vs fx 9-16 "lite"), key each by its representative
    variants_out = {}
    gcount = {}
    for bp in variants:
        gcount[generalise(bp)] = gcount.get(generalise(bp), 0) + 1
    for bp, sels in variants.items():
        variants_out[generalise(bp) if gcount[generalise(bp)] == 1 else bp] = sels

    doc = {
        "format": "wing-tree/1",
        "doc": "See docs/research/wing-edit-schema.md for the format description.",
        "source": {"app": "WING-Edit", "version": info, "sha256": sha,
                   "slice": "arm64", "extractor": "tools/schema-extract/extract.py"},
        "node_count": x.node_count,
        "hash": {
            "algorithm": "murmur3-like-32",
            "input": "getPath() text without its first char, e.g. 'ch.1.flt.lcf' for /ch/1/flt/lcf; "
                     "ASCII a-z upper-cased, spaces skipped",
            "seed": "0xcb21c6be",
            "notes": "4 chars packed big-endian per block (k=(k<<8)|c); the trailing partial block "
                     "gets the full block mix (incl. rotl13,*5+0xe6546b64); final h ^= nchars; "
                     "no fmix. Reference: tools/schema-extract/winghash.py",
            "verified_nodes": f"{total - bad}/{total}",
        },
        "units": x.units,
        "hashes": hashes,
        "snapshot_sections": SECTION_ROOT,
        "tree": tree,
        "variants": variants_out,
        "coverage": {k: {kk: vv for kk, vv in v.items() if kk != "missing"} |
                     {"missing_sample": v["missing"][:200], "missing_count": len(v["missing"])}
                     for k, v in cov.items()},
    }
    os.makedirs(os.path.dirname(a.out) or ".", exist_ok=True)
    with open(a.out, "w") as f:
        json.dump(doc, f, indent=1, ensure_ascii=False)
    if a.coverage_report:
        with open(a.coverage_report, "w") as f:
            json.dump({"coverage": cov, "log": log, "unknown_imports": x.e.unknown_calls}, f, indent=1)
    for k, v in cov.items():
        print(f"coverage {k}: {v['covered']}/{v['total']} = {v['percent']}% "
              f"(via variants {v['via_model_variants']}, missing {len(v['missing'])})", file=sys.stderr)


def generalise(bp):
    return "/".join("*" if s.isdigit() else s for s in bp.split("/"))


def strip_hash_in_arrays(t):
    for c in t.get("children", []):
        if c.get("kind") == "array":
            strip_hash_in_template(c["template"])
        else:
            strip_hash_in_arrays(c)


if __name__ == "__main__":
    main()
