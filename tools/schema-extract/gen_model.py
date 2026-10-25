"""Generate crates/wing-model/src/schema.bin from data/schema/wing-tree.json (T010).

The blob is a compact, little-endian, index-addressed form of the WING parameter tree that
`wing-model` embeds with `include_bytes!` and decodes once at start-up. We chose a checked-in blob
over a build.rs because parsing the 5 MB JSON at build time would need serde_json as a build
dependency and add seconds to every clean build, and a generated .rs with ~43k struct literals
compiles slowly. The blob builds in no time and the Rust side reads it zero-copy for strings.

Inputs (both checked in):
  data/schema/wing-tree.json                         tree, variants, units  (T003)
  data/firmware/json/0006211b-snapshot_11.json       firmware 3.0.6 factory scene (T008), per-instance defaults

Usage:
  python3 -I tools/schema-extract/gen_model.py            # (re)write the blob
  python3 -I tools/schema-extract/gen_model.py --check    # exit 1 if the blob is stale

Format "WMDL" v1 (all little-endian; see crates/wing-model/src/schema.rs for the reader):
  header   b"WMDL", u32 version, u32 counts[9] = strings, descs, enum_items, nodes, groups, models,
           entries, targets, instances
  strings  u32 offsets[n+1], utf-8 bytes, zero padding to 4
  descs    72 B: name,long,fmt,describe u32 (string ids), flags u32, class u8, type u8, unit u8,
           var u8 (1 model param, 2 unused slot, 4 filled from models.md), steps u32, pad u32, min f64, max f64, step f64 (lin step or natural-log step), enum_start u32,
           enum_len u32, maxlen u32, default u32 (raw value; only meaningful for model descriptors)
  enums    16 B: token u32, long u32, fval f64
  nodes    16 B: name u32, long u32, parent u16, first_child u16, next_sibling u16, desc u16
           (0xFFFF = not a parameter / none)
  defaults u32 per node (raw value; string id for string params)
  groups   24 B: key u32, selector u32, model_start u32, model_count u32, target_start u32, target_count u32
  models   20 B: name u32, long u32, enum_index u32, entry_start u32, entry_count u32
  entries  8 B: target u32 (string id of the child name), desc u32
  targets  u32 string id (child names a group can redefine)
  insts    8 B: block node u32, group u32

Raw value encoding: linf/logf/fader -> f32 bits; int -> i32; enum/fenum -> index; string -> string id.
"""
import argparse
import json
import os
import re
import struct
import sys

ROOT = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", ".."))
TREE = os.path.join(ROOT, "data/schema/wing-tree.json")
SCENE = os.path.join(ROOT, "data/firmware/json/0006211b-snapshot_11.json")
OUT = os.path.join(ROOT, "crates/wing-model/src/schema.bin")

TYPES = {"linf": 1, "logf": 2, "fader": 3, "int": 4, "enum": 5, "fenum": 6, "string": 7}
CLASSES = ["CparInt", "CparBool", "CparEnum", "CparLinFloat", "CparLogFloat", "CparLevel", "CparStr",
           "CparFenum", "CparTrigger", "CparVar", "ae::CparTag", "ae::Cstringlist"]
NONE16 = 0xFFFF


class Strings:
    def __init__(self):
        self.ids = {}
        self.list = []

    def id(self, s):
        s = "" if s is None else str(s)
        if s not in self.ids:
            self.ids[s] = len(self.list)
            self.list.append(s)
        return self.ids[s]


def f32_bits(v):
    return struct.unpack("<I", struct.pack("<f", float(v)))[0]


def i32_bits(v):
    return int(v) & 0xFFFFFFFF


def parse_num(text):
    """Parse WING display text: '-40.0', '+3.0', '2k00', '20k0', '-oo'."""
    s = str(text).strip()
    if s in ("-oo", "-inf"):
        return -144.0
    m = re.fullmatch(r"([+-]?\d+)k(\d*)", s)
    if m:
        return float(m.group(1) + "." + (m.group(2) or "0")) * 1000.0
    return float(s)


class Gen:
    def __init__(self, tree, scene):
        self.t = tree
        self.S = Strings()
        self.S.id("")
        self.descs = []
        self.desc_ids = {}
        self.enums = []
        self.units = {u: i + 1 for i, u in enumerate(tree["units"])}
        self.problems = []
        self.filled = []
        self.shadowed = 0
        self.scene = {}
        for sec, pre in tree["snapshot_sections"].items():
            self._flatten(scene.get(sec, {}), pre)
        self.int_fixes = 0
        self._fix_ints(tree["tree"])
        for sels in tree["variants"].values():
            for g in sels.values():
                for m in g["variants"]:
                    for p in m["params"]:
                        self._fix_ints(p)

    def _fix_ints(self, n):
        """T003 parsed int min/max from `describe` but missed ones with a unit ('int [0 .. 100 %]')."""
        if n.get("kind") == "par" and n.get("type") == "int":
            m = re.match(r"int \[(-?\d+) \.\. (-?\d+)", n.get("describe") or "")
            if m and (int(m.group(1)), int(m.group(2))) != (n.get("min"), n.get("max")):
                n["min"], n["max"] = int(m.group(1)), int(m.group(2))
                self.int_fixes += 1
        for c in n.get("children") or []:
            self._fix_ints(c)
        if n.get("kind") == "array":
            self._fix_ints(n["template"])

    def _flatten(self, x, p):
        if isinstance(x, dict):
            for k, v in x.items():
                self._flatten(v, (p + "/" if p else "") + k)
        else:
            self.scene[p] = x

    # ---- descriptors -------------------------------------------------------------------------
    def desc(self, p, variant=False, filled=False):
        typ = TYPES[p["type"]]
        enum_items = []
        if p["type"] == "enum":
            longs = p.get("long_values") or []
            for i, tok in enumerate(p.get("values") or []):
                enum_items.append((tok, longs[i] if i < len(longs) else tok, 0.0))
        elif p["type"] == "fenum":
            labels = p.get("labels") or []
            for i, fv in enumerate(p.get("values") or []):
                enum_items.append((str(fv), labels[i] if i < len(labels) else str(fv), float(fv)))
        step = p.get("step", p.get("logstep", 0.0)) or 0.0
        unit = self.units.get(p.get("unit"), 0)
        default = self.encode(p, p.get("default"), model_default=True) if variant else 0
        key = (p["name"] if variant else "", p.get("long") or "", p.get("fmt") or "", int(p["flags"], 16),
               p["class"], typ, unit, variant, bool(p.get("unused_slot")), p.get("steps", 0) or 0,
               float(p.get("min", 0) or 0), float(p.get("max", 0) or 0), float(step), tuple(enum_items),
               p.get("maxlen", 0) or 0, default, bool(filled), p.get("describe") or "")
        if key in self.desc_ids:
            return self.desc_ids[key]
        start = len(self.enums)
        for tok, lng, fv in enum_items:
            self.enums.append((self.S.id(tok), self.S.id(lng), fv))
        var = (1 if variant else 0) | (2 if p.get("unused_slot") else 0) | (4 if filled else 0)
        rec = struct.pack("<IIIIIBBBBIIdddIIII", self.S.id(key[0]), self.S.id(key[1]), self.S.id(key[2]),
                          self.S.id(key[17]), key[3], CLASSES.index(p["class"]), typ, unit, var, key[9], 0,
                          key[10], key[11], key[12], start, len(enum_items), key[14], default)
        assert len(rec) == 72
        self.desc_ids[key] = len(self.descs)
        self.descs.append(rec)
        return self.desc_ids[key]

    def encode(self, p, v, model_default=False, where=""):
        """Encode a default/scene value to the raw u32 for parameter definition p."""
        typ = p["type"]
        try:
            if typ in ("linf", "logf", "fader"):
                if v is None:
                    x = -144.0 if typ == "fader" else min(max(0.0, p.get("min", 0.0)), p.get("max", 0.0))
                else:
                    x = parse_num(v) if isinstance(v, str) else float(v)
                return f32_bits(x)
            if typ == "int":
                if v is None:
                    return i32_bits(min(max(0, p.get("min", 0)), p.get("max", 0)))
                if isinstance(v, bool):
                    return int(v)
                if isinstance(v, str):
                    v = {"true": 1, "false": 0}.get(v.lower(), v)
                return i32_bits(int(float(v)))
            if typ == "enum":
                vals = p.get("values") or []
                if v is None or not vals:
                    return 0 if v is None or not isinstance(v, int) else v
                if isinstance(v, bool):
                    v = "ON" if v else "OFF"
                sv = str(v)
                if sv in vals:
                    return vals.index(sv)
                longs = p.get("long_values") or []
                if sv in longs:
                    return longs.index(sv)
                for i, tok in enumerate(vals):
                    if tok.lower() == sv.lower():
                        return i
                # pardef default strings are truncated to 15 chars (VSS3 'Medium Vocal Ha')
                pre = [i for i, tok in enumerate(vals) if len(sv) >= 15 and tok.startswith(sv)]
                if len(pre) == 1:
                    return pre[0]
                raise ValueError(f"unknown enum token {sv!r}")
            if typ == "fenum":
                vals = p.get("values") or []
                if v is None:
                    return 0
                x = parse_num(v) if isinstance(v, str) else float(v)
                return min(range(len(vals)), key=lambda i: abs(vals[i] - x))
            if typ == "string":
                return self.S.id("" if v is None else v)
        except (ValueError, TypeError) as e:
            self.problems.append(f"default {where or p.get('name')}={v!r}: {e}")
            return 0
        raise ValueError(typ)

    # ---- tree --------------------------------------------------------------------------------
    def build(self):
        self.nodes = []  # [name, long, parent, first_child, next_sibling, desc, path, tmpl, default]
        root = self.t["tree"]
        self.nodes.append(["", root.get("long", ""), NONE16, NONE16, NONE16, NONE16, "", id(root), 0])
        self.tmpl_of = {}
        self._expand(root, 0, "")
        self.by_path = {n[6]: i for i, n in enumerate(self.nodes)}
        assert len(self.nodes) < NONE16
        self.build_variants()
        for i, n in enumerate(self.nodes):
            if n[5] != NONE16 and n[8] is None:
                n[8] = 0

    def _expand(self, node, idx, path):
        prev = None
        for c in node.get("children") or []:
            if c["kind"] == "array":
                tp = c["template"]
                longs = c.get("longs") or []
                kids = [(str(i), tp, longs[i - c["first"]] if i - c["first"] < len(longs) else tp.get("long", ""))
                        for i in range(c["first"], c["last"] + 1)]
            else:
                kids = [(c["name"], c, c.get("long", ""))]
            for name, tn, lng in kids:
                p = (path + "/" if path else "") + name
                me = len(self.nodes)
                desc = NONE16
                default = None
                if tn["kind"] == "par":
                    desc = self.desc(tn)
                    sv = self.scene.get(p, tn.get("default"))
                    default = self.encode(tn, sv, where=p)
                self.nodes.append([name, lng, idx, NONE16, NONE16, desc, p, id(tn), default])
                self.tmpl_of[p] = tn
                if prev is None:
                    self.nodes[idx][3] = me
                else:
                    self.nodes[prev][4] = me
                prev = me
                self._expand(tn, me, p)

    def children(self, i):
        c = self.nodes[i][3]
        while c != NONE16:
            yield c
            c = self.nodes[c][4]

    def child(self, i, name):
        for c in self.children(i):
            if self.nodes[c][0] == name:
                return c
        return None

    # ---- variants ----------------------------------------------------------------------------
    # T003's extractor dropped a few FX model params whose names equal an existing slot index
    # (reported by T004, docs/research/models.md §6). Restore them from a sibling param of the same
    # model with an identical range per models.md; the descriptor is flagged provenance=models.md.
    # (key prefix, model, slot, name, long, sibling to copy the definition from)
    FILLS = [
        ("fx/", "GEQ", 5, "40", None, "50"),     # 31-band GEQ 40 Hz band, -15..15 dB like its neighbours
        ("fx/", "PIA", 3, "31", "31", "63"),     # PIA 560 31 Hz band, -12..12 dB (models.md PIA table)
        ("fx/", "MACH4", 3, "40", "40", "160"),  # MACH EQ4 40 Hz band, -5..5 (models.md MACH4 table)
        ("fx/", "*MASTER*", 7, "40", "40", "160"),  # *MASTER* chain's MACH4 section (models.md *MASTER*)
    ]

    def fill_missing(self, key, g):
        for prefix, model, slot, name, lng, sib in self.FILLS:
            if not key.startswith(prefix):
                continue
            for m in g["variants"]:
                if m["name"] != model or any(p.get("slot") == slot for p in m["params"]):
                    continue
                src = next(p for p in m["params"] if p["name"] == sib)
                p = dict(src, name=name, path=name, slot=slot, _filled=True)
                if lng is not None or "long" in src:
                    p["long"] = lng if lng is not None else name
                p["hash"] = self.t["hashes"][g["representative"] + "/" + str(slot)]
                m["params"].append(p)
                m["params"].sort(key=lambda q: q.get("slot", 0))
                self.filled.append(f"{key}:{model}/{name}@slot{slot}")
        # report remaining slot gaps so new extractor drift is visible
        for m in g["variants"]:
            slots = sorted(p["slot"] for p in m["params"] if "slot" in p)
            if slots and slots != list(range(1, len(slots) + 1)):
                self.problems.append(f"slot gap {key}:{m['name']} {slots}")

    def build_variants(self):
        from wing_hash_ref import wing_hash  # noqa: E402  (tools/schema-extract/winghash.py)
        hashes = self.t["hashes"]
        self.groups, self.models, self.entries, self.targets, self.insts = [], [], [], [], []
        claims = {}
        for key, sels in self.t["variants"].items():
            for sel_name, g in sels.items():
                rep = g["representative"]
                rep_id = self.by_path[rep]
                sel_rel = g["selector"][len(rep) + 1:]
                assert "/" not in sel_rel and self.child(rep_id, sel_rel) is not None, g["selector"]
                rep_tmpl = self.nodes[rep_id][7]
                kpat = key.split("/")
                insts = []
                for i, n in enumerate(self.nodes):
                    if n[7] == rep_tmpl:
                        segs = n[6].split("/")
                        literal = len(segs) == len(kpat) and all(a == "*" or a == b for a, b in zip(kpat, segs))
                        insts.append((i, literal))
                gidx = len(self.groups)
                targets = []
                mstart = len(self.models)
                sel_def = self.tmpl_of[g["selector"]]
                self.fill_missing(key, g)
                for m in g["variants"]:
                    estart = len(self.entries)
                    for p in m["params"]:
                        tgt = str(p["slot"]) if "slot" in p else p["path"]
                        assert self.child(rep_id, tgt) is not None, (key, m["name"], tgt)
                        want = hashes.get(rep + "/" + tgt)
                        if want is None or int(p["hash"], 16) != int(want, 16) or wing_hash(rep + "/" + tgt) != int(want, 16):
                            self.problems.append(f"variant hash mismatch {key} {m['name']} {tgt}")
                        # numeric model param names (GEQ '20', PIA '31', ...) can equal a slot node name;
                        # the Rust lookup gives the active model's names precedence for text paths.
                        other = self.child(rep_id, p["name"])
                        if other is not None and self.nodes[other][0] != tgt:
                            self.shadowed += 1
                        if tgt not in targets:
                            targets.append(tgt)
                        self.entries.append((self.S.id(tgt), self.desc(p, variant=True, filled=p.get("_filled"))))
                    self.models.append((self.S.id(m["name"]), self.S.id(m.get("long") or ""), m["index"], estart,
                                        len(self.entries) - estart))
                    assert sel_def["values"][m["index"]] == m["name"], (key, m["name"])
                tstart = len(self.targets)
                self.targets.extend(self.S.id(x) for x in targets)
                self.groups.append((self.S.id(key), self.S.id(sel_rel), mstart, len(g["variants"]), tstart,
                                    len(targets), g, targets))
                for i, literal in insts:
                    claims.setdefault((i, sel_rel), []).append((literal, gidx))
        for (i, sel_rel), cl in sorted(claims.items()):
            lit = [g for literal, g in cl if literal]
            if len(cl) == 1:
                gidx = cl[0][1]
            elif len(lit) == 1:
                gidx = lit[0]
            else:
                self.problems.append(f"ambiguous variant groups for {self.nodes[i][6]}: {cl}")
                continue
            self.insts.append((i, gidx))
        self.insts.sort()
        # initial model per instance: per-instance defaults for the active model's targets
        for i, gidx in self.insts:
            key_s, sel_s, mstart, mcount, tstart, tcount, g, targets = self.groups[gidx]
            sel_rel = self.S.list[sel_s]
            sel_node = self.child(i, sel_rel)
            active = self.nodes[sel_node][8]
            model = next((m for m in g["variants"] if m["index"] == active), None)
            block_path = self.nodes[i][6]
            for tgt in targets:
                tn = self.child(i, tgt)
                p = None
                if model is not None:
                    p = next((q for q in model["params"] if (str(q["slot"]) if "slot" in q else q["path"]) == tgt), None)
                if p is None:
                    continue
                sk = block_path + "/" + p["name"]
                v = self.scene[sk] if sk in self.scene else p.get("default")
                self.nodes[tn][8] = self.encode(p, v, where=sk)

    # ---- output ------------------------------------------------------------------------------
    def blob(self):
        out = bytearray()
        for n in self.nodes:  # intern every string before the table is written
            self.S.id(n[0])
            self.S.id(n[1])
        S = self.S.list
        counts = [len(S), len(self.descs), len(self.enums), len(self.nodes), len(self.groups), len(self.models),
                  len(self.entries), len(self.targets), len(self.insts)]
        out += b"WMDL" + struct.pack("<I", 1) + struct.pack("<9I", *counts)
        enc = [s.encode("utf-8") for s in S]
        off = 0
        offs = [0]
        for b in enc:
            off += len(b)
            offs.append(off)
        out += struct.pack(f"<{len(offs)}I", *offs)
        out += b"".join(enc)
        while len(out) % 4:
            out.append(0)
        for d in self.descs:
            out += d
        for tok, lng, fv in self.enums:
            out += struct.pack("<IId", tok, lng, fv)
        for n in self.nodes:
            out += struct.pack("<IIHHHH", self.S.id(n[0]), self.S.id(n[1]), n[2], n[3], n[4], n[5])
        for n in self.nodes:
            out += struct.pack("<I", (n[8] or 0) & 0xFFFFFFFF)
        for g in self.groups:
            out += struct.pack("<6I", *g[:6])
        for m in self.models:
            out += struct.pack("<5I", *m)
        for e in self.entries:
            out += struct.pack("<2I", *e)
        for t in self.targets:
            out += struct.pack("<I", t)
        for i, g in self.insts:
            out += struct.pack("<2I", i, g)
        return bytes(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="fail if the checked-in blob is stale")
    ap.add_argument("--out", default=OUT)
    a = ap.parse_args()
    sys.dont_write_bytecode = True
    sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
    import winghash
    sys.modules["wing_hash_ref"] = winghash
    with open(TREE) as f:
        tree = json.load(f)
    with open(SCENE) as f:
        scene = json.load(f)
    g = Gen(tree, scene)
    g.build()
    blob = g.blob()
    params = sum(1 for n in g.nodes if n[5] != NONE16)
    print(f"nodes={len(g.nodes)} params={params} descs={len(g.descs)} enum_items={len(g.enums)} "
          f"strings={len(g.S.list)} groups={len(g.groups)} models={len(g.models)} entries={len(g.entries)} "
          f"instances={len(g.insts)} blob={len(blob)} bytes")
    print(f"filled from models.md: {g.filled}; int ranges re-parsed from describe: {g.int_fixes}")
    print(f"model param names equal to another slot's name: {g.shadowed}")
    for p in g.problems[:40]:
        print("WARN", p)
    if len(g.problems) > 40:
        print(f"WARN ... {len(g.problems) - 40} more")
    if a.check:
        with open(a.out, "rb") as f:
            if f.read() != blob:
                print("schema.bin is stale; rerun tools/schema-extract/gen_model.py")
                sys.exit(1)
        print("schema.bin is up to date")
        return
    with open(a.out, "wb") as f:
        f.write(blob)


if __name__ == "__main__":
    main()
