"""WING node hash, re-implemented from ncl::Cnode::setRoot (WING-Edit 3.3.4).

Input is the node path as printed by ncl::Cnode::getPath(stream, 0), e.g.
"/ch.1.flt.lcf"; the first character is skipped.  It is a MurmurHash3-x86-32
variant: characters are upper-cased (a-z only), spaces are skipped, 4 chars
are packed big-endian into a block, a trailing partial block receives the full
block mix, the final step is h ^= number_of_chars, and there is no fmix.
"""
M32 = 0xFFFFFFFF


def _rotl(x, r):
    return ((x << r) | (x >> (32 - r))) & M32


def wing_hash(path: str) -> int:
    """path: getPath() form ('/ch.1.flt.lcf') or slash form ('ch/1/flt/lcf')."""
    if "/" in path[1:] or not path.startswith("/"):
        path = "/" + path.strip("/").replace("/", ".")
    b = path.encode("latin-1")
    if len(b) < 2:
        return 0
    h = 0xCB21C6BE
    total = 0
    k = 0
    cnt = 0
    for ch in b[1:]:
        if 0x61 <= ch <= 0x7A:
            ch -= 0x20
        if ch == 0x20:
            continue
        k = ((k << 8) | ch) & M32
        cnt += 1
        total += 1
        if cnt == 4:
            h = _mix(h, k)
            k = 0
            cnt = 0
    if cnt or total == 0:
        h = _mix(h, k)
    return (h ^ total) & M32


def _mix(h, k):
    k = (k * 0xCC9E2D51) & M32
    k = _rotl(k, 15)
    k = (k * 0x1B873593) & M32
    h ^= k
    h = _rotl(h, 13)
    return (h * 5 + 0xE6546B64) & M32


if __name__ == "__main__":
    import sys
    for p in sys.argv[1:]:
        print(f"{wing_hash(p):08x}  {p}")
