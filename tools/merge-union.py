#!/usr/bin/env python3
"""Resolve git conflict markers in a file by keeping both sides (ours then theirs). For additive files like Cargo.toml."""
import re, sys
for p in sys.argv[1:]:
    s = open(p).read()
    s = re.sub(r'<<<<<<< [^\n]*\n(.*?)=======\n(.*?)>>>>>>> [^\n]*\n', lambda m: m.group(1) + m.group(2), s, flags=re.S)
    open(p, 'w').write(s)
