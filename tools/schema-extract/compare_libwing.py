#!/usr/bin/env python3
"""Cross-check data/schema/wing-tree.json (WING-Edit 3.3.4) against libwing's embedded node definitions (fw 3.0.5).

Usage:  python3 -I tools/schema-extract/compare_libwing.py LIBWING_DIR [--tree data/schema/wing-tree.json]
                                                           [--json OUT.json] [--examples N]

LIBWING_DIR is a checkout of https://github.com/dannydulai/libwing (Apache-2.0).  Nothing from it is copied
into the repo; this script only reads src/propmap.rs (binary records) and cross-checks propmap.jsonl if present.

Join strategy
  1. Default tree: every wing-tree node/param is expanded (arrays unrolled) to a full path.  A libwing record with
     the same fullname is a path match; ids are then compared (libwing id is int32, ours is the u32 hash).
  2. Variants: libwing names model-dependent params /<block>/<MODEL>/<param> (id = hash of /<block>/<slot>); these
     are resolved against wing-tree `variants` (block pattern, model name, param path).
  3. Leftovers are joined by id only, to separate "renamed" from "really missing".
"""
import argparse
import json
import math
import os
import re
import struct
import sys
from collections import Counter, defaultdict

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))  # -I removes the script dir
from winghash import wing_hash  # noqa: E402

LW_TYPES = {0: "node", 1: "linear float", 2: "log float", 3: "fader level", 4: "integer",
            5: "string enum", 6: "float enum", 7: "string"}
LW_UNITS = {0: "", 1: "dB", 2: "%", 3: "ms", 4: "Hz", 5: "m", 6: "s", 7: "octaves"}
WE_TYPES = {"linf": "linear float", "logf": "log float", "fader": "fader level", "int": "integer",
            "enum": "string enum", "fenum": "float enum", "string": "string"}
SHOW = ("type", "unit", "long", "min", "max", "steps", "maxlen")


# ---------------------------------------------------------------- libwing
def parse_def(raw):
    p = 0

    def u16():
        nonlocal p
        v = struct.unpack_from(">H", raw, p)[0]; p += 2; return v

    def i32():
        nonlocal p
        v = struct.unpack_from(">i", raw, p)[0]; p += 4; return v

    def f32():
        nonlocal p
        v = struct.unpack_from(">f", raw, p)[0]; p += 4; return v

    def s8():
        nonlocal p
        n = raw[p]; p += 1
        v = raw[p:p + n].decode("utf-8"); p += n; return v

    d = {"parent": i32(), "id": i32(), "index": u16(), "name": s8(), "long": s8()}
    flags = u16()
    d["type"] = LW_TYPES.get((flags >> 4) & 15, "node")
    d["unit"] = LW_UNITS.get(flags & 15, "")
    d["ro"] = bool((flags >> 9) & 1)
    t = d["type"]
    if t == "string":
        d["maxlen"] = u16()
    elif t in ("linear float", "log float"):
        d["min"], d["max"], d["steps"] = f32(), f32(), i32()
    elif t == "integer":
        d["min"], d["max"] = i32(), i32()
    elif t == "string enum":
        d["items"] = []
        for _ in range(u16()):
            a = s8(); b = s8(); d["items"].append((a, b))
    elif t == "float enum":
        d["items"] = []
        for _ in range(u16()):
            a = f32(); b = s8(); d["items"].append((a, b))
    return d


def load_libwing(root):
    src = open(os.path.join(root, "src", "propmap.rs"), encoding="latin-1").read()
    i = src.index('let d = b"') + len('let d = b"')
    j = src.index('";', i)
    blob = bytes(int(h, 16) for h in re.findall(r"\\x([0-9A-Fa-f]{2})", src[i:j]))
    recs, p = {}, 0
    while p < len(blob):
        p += 1  # is_fake byte
        n = struct.unpack_from(">H", blob, p)[0]; p += 2
        name = blob[p:p + n].decode("utf-8"); p += n
        n = struct.unpack_from(">H", blob, p)[0]; p += 2
        recs[name] = parse_def(blob[p:p + n]); p += n
    jl = os.path.join(root, "propmap.jsonl")
    check = None
    if os.path.exists(jl):
        ids = {}
        for line in open(jl):
            r = json.loads(line); ids[r["fullname"]] = r["id"]
        check = (len(ids), ids == {k: v["id"] for k, v in recs.items()})
    return recs, check


# ---------------------------------------------------------------- wing-tree
def flatten(tree):
    """-> {path: node dict}; arrays unrolled; path has no leading slash."""
    out = {}

    def walk(n, path):
        if n["kind"] == "array":
            longs = n.get("longs") or []
            for k in range(n["first"], n["last"] + 1):
                t = dict(n["template"])
                if 0 <= k - n["first"] < len(longs):
                    t["long"] = longs[k - n["first"]]  # per-instance display name (A1, ...)
                walk(t, path + [str(k)])
            return
        out["/".join(path)] = n
        for c in n.get("children", []):
            if c["kind"] == "array":
                walk(c, path)
            else:
                walk(c, path + [c["name"]])

    for c in tree["children"]:
        walk(c, [] if c["kind"] == "array" else [c["name"]])
    return out


def we_def(n):
    """normalise a wing-tree node/par into the same shape as the libwing dict."""
    if n["kind"] == "node":
        return {"type": "node", "long": n.get("long", "")}
    d = {"type": WE_TYPES.get(n["type"], n["type"]), "long": n.get("long", ""), "unit": n.get("unit", "")}
    t = n["type"]
    if t in ("linf", "logf", "fader"):
        d["min"], d["max"], d["steps"] = n["min"], n["max"], n["steps"]
    elif t == "int":
        if "min" in n:
            d["min"], d["max"] = n["min"], n["max"]
        else:  # a few ints only carry the describe text, e.g. "int [0 .. 100 %]"
            m = re.match(r"int \[\s*(-?\d+) \.\. (-?\d+)", n.get("describe", ""))
            d["min"], d["max"] = int(m.group(1)), int(m.group(2))
    elif t == "string":
        d["maxlen"] = n.get("maxlen")
    elif t == "enum":
        lv = n.get("long_values") or [""] * len(n["values"])
        d["items"] = list(zip(n["values"], lv))
    elif t == "fenum":
        lb = n.get("labels") or [""] * len(n["values"])
        d["items"] = list(zip(n["values"], lb))
    return d


def resolve_variant(path, matchers):
    """'ch/1/dyn/COMP/thr' -> (block pattern, model, param path, variant dict) or None."""
    seg = path.split("/")
    for pat, v in matchers:
        n = len(pat)
        if len(seg) <= n + 1:
            continue
        if pat[0] == "fx" and n == 2 and pat[1].isdigit() and seg[0] == "fx" and seg[1].isdigit():
            ok = int(pat[1]) <= int(seg[1]) <= int(pat[1]) + 7  # 'fx/1' template = fx 1-8, 'fx/9' = fx 9-16
        else:
            ok = all(a == b or a == "*" for a, b in zip(pat, seg[:n]))
        if ok:
            for var in v["variants"]:
                if var["name"] == seg[n]:
                    return "/".join(pat), seg[n], "/".join(seg[n + 1:]), var
    return None


# ---------------------------------------------------------------- compare
def close(a, b):
    return a == b or math.isclose(a, b, rel_tol=2e-3, abs_tol=1e-6)


def diff_def(lw, we):
    """-> list of mismatching field names (libwing def vs wing def)."""
    if lw["type"] != we["type"]:
        return ["type"]
    out = []
    if lw["unit"] != we.get("unit", ""):
        out.append("unit")
    if lw["long"] != we.get("long", ""):
        out.append("longname")
    t = lw["type"]
    if t in ("linear float", "log float"):
        if lw["steps"] != we["steps"]:
            out.append("steps")
        if not (close(lw["min"], we["min"]) and close(lw["max"], we["max"])):
            out.append("range")
    elif t == "integer":
        if (lw["min"], lw["max"]) != (we["min"], we["max"]):
            out.append("range")
    elif t == "string":
        if lw["maxlen"] != we["maxlen"]:
            out.append("maxlen")
    elif t in ("string enum", "float enum"):
        a = [x[0] for x in lw["items"]]; b = [x[0] for x in we["items"]]
        if t == "float enum":
            same = len(a) == len(b) and all(close(x, y) for x, y in zip(a, b))
            if not same:
                out.append("enum_values")
        elif a != b:
            out.append("enum_values" if set(a) != set(b) else "enum_order")
        if "enum_values" not in out and "enum_order" not in out:
            if any(x[1] and y[1] and x[1] != y[1] for x, y in zip(lw["items"], we["items"])):
                out.append("enum_labels")
    return out


def brief(d, f):
    if f == "enum_labels":
        return [x for x in d.get("items", [])][:6]
    if "enum" in f:
        return [x[0] for x in d.get("items", [])][:14]
    return {k: d[k] for k in SHOW if d.get(k) not in (None, "")}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("libwing")
    ap.add_argument("--tree", default=os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "..",
                                                    "data", "schema", "wing-tree.json"))
    ap.add_argument("--json", help="write counts + examples here")
    ap.add_argument("--examples", type=int, default=6)
    a = ap.parse_args()
    N = a.examples

    lw, check = load_libwing(a.libwing)
    print(f"libwing: {len(lw)} records parsed from src/propmap.rs; jsonl cross-check (count, same ids): {check}")
    tree = json.load(open(a.tree))
    flat = flatten(tree["tree"])
    hashes = tree["hashes"]
    print(f"wing-tree: {len(flat)} expanded paths (node_count {tree['node_count']} incl. root)")
    bad = [p for p in flat if int(hashes[p], 16) != wing_hash(p)]
    print(f"winghash vs tree hashes: {len(flat) - len(bad)}/{len(flat)} ok")
    matchers = [(blk.split("/"), v) for blk, sels in tree["variants"].items() for v in sels.values()]

    R = defaultdict(int)
    EX = defaultdict(list)

    FAM = defaultdict(Counter)

    def fam_of(item):
        path = item[0] if isinstance(item, tuple) else item
        return re.sub(r"(?<=/)\d+(?=/|$)", "#", str(path))

    def note(key, item):
        R[key] += 1
        FAM[key][fam_of(item)] += 1
        if len(EX[key]) < N:
            EX[key].append(item)

    used = set()
    # ---- 1. default-tree path join
    for p, n in flat.items():
        r = lw.get("/" + p)
        h = int(hashes[p], 16)
        if r is None:
            note("wing_only_unused_slot" if n.get("unused_slot") else "wing_only_path", p)
            continue
        used.add("/" + p)
        R["path_matched"] += 1
        if (r["id"] & 0xFFFFFFFF) == h:
            R["id_ok"] += 1
        else:
            note("id_mismatch", (p, hex(r["id"] & 0xFFFFFFFF), hex(h)))
        d = we_def(n)
        mm = diff_def(r, d)
        if not mm:
            R["def_identical"] += 1
        else:
            R["def_mismatch_records"] += 1
        for f in mm:
            note("field_" + f, (p, {"libwing": brief(r, f), "wing": brief(d, f)}))
        if r["type"] != "node":
            R["params_matched"] += 1
            R["ro_%s_dollar_%s" % (r["ro"], n["name"].startswith("$"))] += 1

    # ---- 2. variants
    seen = set()
    for path, r in lw.items():
        if path in used:
            continue
        res = resolve_variant(path[1:], matchers)
        if not res:
            continue
        used.add(path)
        blk, model, ppath, var = res
        R["variant_records"] += 1
        pm = next((x for x in var["params"] if x.get("path", x["name"]) == ppath), None)
        if pm is None:
            note("variant_record_param_absent_in_wing", path)
            continue
        seen.add((blk, model, ppath))
        seg = path[1:].split("/")
        want = wing_hash("/".join(seg[:len(blk.split("/"))]) + "/" + str(r["index"]))
        if (r["id"] & 0xFFFFFFFF) == want:
            R["variant_id_ok"] += 1
        else:
            note("variant_id_mismatch", (path, hex(r["id"] & 0xFFFFFFFF), hex(want)))
        if "slot" in pm and pm["slot"] != r["index"]:
            note("variant_slot_mismatch", (path, r["index"], pm["slot"]))
        d = we_def(pm)
        mm = diff_def(r, d)
        if not mm:
            R["variant_def_identical"] += 1
        else:
            R["variant_def_mismatch_records"] += 1
        for f in mm:
            note("vfield_" + f, (path, {"libwing": brief(r, f), "wing": brief(d, f)}))
    for blk, sels in tree["variants"].items():
        for v in sels.values():
            for var in v["variants"]:
                for pm in var["params"]:
                    R["variant_params_wing_total"] += 1
                    if (blk, var["name"], pm.get("path", pm["name"])) not in seen:
                        note("variant_param_wing_only", f"{blk}/{var['name']}/{pm.get('path', pm['name'])}")

    # ---- 3. leftovers (libwing records matched neither way)
    wing_by_id = defaultdict(list)
    for p in flat:
        wing_by_id[int(hashes[p], 16)].append(p)
    left = [p for p in lw if p not in used]
    cls = Counter()
    cex = defaultdict(list)
    for pth in left:
        hid = lw[pth]["id"] & 0xFFFFFFFF
        c = "id_in_wing_under_other_path" if hid in wing_by_id else "no_wing_counterpart"
        cls[(c, pth.split("/")[1])] += 1
        if len(cex[c]) < 3 * N:
            cex[c].append(pth if c == "no_wing_counterpart" else (pth, wing_by_id[hid][:2]))

    # ---- report
    print(f"\nlibwing records: {len(lw)} (nodes {sum(1 for r in lw.values() if r['type'] == 'node')}, "
          f"params {sum(1 for r in lw.values() if r['type'] != 'node')})")
    print("\n== counters ==")
    for k in sorted(R):
        print(f"  {k}: {R[k]}")
    print(f"\n== libwing records matched by neither default path nor variant: {len(left)} ==")
    for (c, top), n in sorted(cls.items(), key=lambda x: -x[1]):
        print(f"    {c:30s} /{top:8s} {n}")
    print("\n== top path families per category (numbers folded to #) ==")
    for k in sorted(FAM):
        print(f"  -- {k}: {R[k]} records in {len(FAM[k])} families")
        for f, c in FAM[k].most_common(6):
            print(f"      {c:6d}  {f}")
    lf = Counter()
    for pth in left:
        lf[re.sub(r"(?<=/)\d+(?=/|$)", "#", "/".join(pth.split("/")[:4]))] += 1
    print("  -- unmatched libwing families (first 3 segments)")
    for f, c in lf.most_common(12):
        print(f"      {c:6d}  {f}")
    print("\n== examples ==")
    for k in sorted(EX):
        print(f"  -- {k}")
        for e in EX[k]:
            print("      ", e)
    for c, v in cex.items():
        print("  -- leftover examples:", c)
        for e in v:
            print("      ", e)
    if a.json:
        json.dump({"counts": dict(R), "examples": {k: [str(e) for e in v] for k, v in EX.items()},
                   "leftover_classes": {f"{c}|{t}": n for (c, t), n in cls.items()}}, open(a.json, "w"), indent=1)


if __name__ == "__main__":
    main()
