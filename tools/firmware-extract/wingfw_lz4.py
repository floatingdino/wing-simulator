#!/usr/bin/env python3
"""Decompress the LZ4 region of a Behringer WING .wingfw image and harvest clear-text assets (T008).

Read-only on the firmware. Pure standard library. Run with:  python3 -I wingfw_lz4.py FIRMWARE OUTDIR

Findings (see docs/research/firmware.md, "T008"):
  * From file offset 0x51EC69 to ~0x2512BBD (67 high-entropy bytes remain before EOF: likely a signature, untouched) the image is ONE continuous raw LZ4 *block-format* sequence stream
    (token | literals | u16le offset | match-length ext), no framing/headers, matches may reach 64 KiB back across
    what are really many logical records ("linked blocks").  It is compression only, not encryption.
  * The true stream start is not marked.  We start at a token boundary near the beginning (0x51EC69) with a
    synthetic 64 KiB history; the first ~64 KiB of output may therefore contain bytes copied from that synthetic
    history (reported, and never used).  Everything after that is exact.
  * Decoded payload (~44.7 MB) is mostly UI/graphics assets plus length-prefixed JSON records:
      u32le length, then JSON text.  The big one is the factory default scene (snapshot.11, 315,265 bytes);
      8 small `snapshot.4` DAW-button templates precede it.
"""
import json
import re
import sys
from pathlib import Path

START = 0x51EC69          # first token boundary of the stream (best-scoring sync point)
PAD = 65536               # synthetic history so early back-references resolve
PADBYTE = 0x00


def lz4_stream(buf: bytes, start: int, end: int, pad: int = PAD):
    """Decode raw LZ4 sequences from buf[start:end]. Returns (output_without_pad, stop_offset, reason)."""
    out = bytearray(bytes([PADBYTE]) * pad)
    i = start
    reason = 'end'
    while i < end:
        tok = buf[i]
        i += 1
        lit = tok >> 4
        if lit == 15:
            while True:
                b = buf[i]
                i += 1
                lit += b
                if b != 255:
                    break
        if i + lit > end:
            reason = 'litover'
            i -= 1
            break
        out += buf[i:i + lit]
        i += lit
        if i + 2 > end:
            reason = 'tail'
            break
        off = buf[i] | (buf[i + 1] << 8)
        i += 2
        mlen = tok & 15
        if mlen == 15:
            while True:
                b = buf[i]
                i += 1
                mlen += b
                if b != 255:
                    break
        mlen += 4
        n = len(out)
        if off == 0 or off > n:
            reason = 'badoffset'
            i -= 3
            break
        if off >= mlen:
            out += out[n - off:n - off + mlen]
        else:  # overlapping copy: repeat the period
            period = bytes(out[n - off:n])
            out += (period * (mlen // off + 1))[:mlen]
    return bytes(out[pad:]), i, reason


def harvest_json(data: bytes, outdir: Path):
    """Find u32le-length-prefixed JSON records and write each valid one."""
    found = []
    for m in re.finditer(rb'\{(?:"|\n)', data):
        i = m.start()
        if i < 4:
            continue
        n = int.from_bytes(data[i - 4:i], 'little')
        if 20 < n < 4_000_000 and i + n <= len(data):
            try:
                obj = json.loads(data[i:i + n])
            except Exception:
                continue
            if isinstance(obj, dict) and 'type' in obj:
                found.append((i, n, obj['type']))
                name = '%08x-%s.json' % (i, str(obj['type']).replace('.', '_'))
                (outdir / name).write_bytes(data[i:i + n])
    return found


def main():
    fw = Path(sys.argv[1])
    outdir = Path(sys.argv[2])
    outdir.mkdir(parents=True, exist_ok=True)
    buf = fw.read_bytes()
    data, stop, why = lz4_stream(buf, START, len(buf))
    print('decoded %d bytes from 0x%x to 0x%x (%s); first %d output bytes unreliable' % (len(data), START, stop, why, PAD))
    recs = harvest_json(data, outdir)
    for i, n, t in recs:
        print('  json @out+0x%x len %d type %s%s' % (i, n, t, '  (UNRELIABLE: <64KiB)' if i < PAD else ''))
    if '--dump' in sys.argv:
        (outdir / 'decoded.bin').write_bytes(data)  # ~44 MB; keep out of git


if __name__ == '__main__':
    main()
