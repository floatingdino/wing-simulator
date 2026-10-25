#!/usr/bin/env python3
"""Resolve git conflict markers by keeping both sides (ours then theirs). For additive files (Cargo.toml,
mod lists, registration tables). Also splits doubled `[[bench]]` names into separate bench blocks."""
import re, sys
for p in sys.argv[1:]:
    s = open(p).read()
    s = re.sub(r'<<<<<<< [^\n]*\n(.*?)=======\n(.*?)>>>>>>> [^\n]*\n', lambda m: m.group(1) + m.group(2), s, flags=re.S)
    if p.endswith('.toml'):
        while True:
            n = re.sub(r'(name = "[^"]+"\n)(name = "[^"]+"\n)(harness = false)', r'\1\3\n\n[[bench]]\n\2\3', s)
            if n == s:
                break
            s = n
    open(p, 'w').write(s)
