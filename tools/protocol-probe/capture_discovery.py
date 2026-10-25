#!/usr/bin/env python3
"""Passively capture WING discovery probes on UDP 2222 (no root needed).

Binds 0.0.0.0:2222 with SO_REUSEADDR/SO_REUSEPORT and hexdumps every datagram
(including broadcasts) with source address and timestamp. Does NOT reply.

Usage: python3 capture_discovery.py [--seconds N] [--port 2222]
"""
import argparse
import socket
import sys
import time


def hexdump(data: bytes) -> str:
    lines = []
    for off in range(0, len(data), 16):
        chunk = data[off:off + 16]
        hx = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        lines.append(f"{off:08x}  {hx:<47}  |{asc}|")
    return "\n".join(lines)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--port", type=int, default=2222)
    a = ap.parse_args()
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    if hasattr(socket, "SO_REUSEPORT"):
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
    s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
    ip_recvdstaddr = getattr(socket, "IP_RECVDSTADDR", 7)  # macOS value 7
    s.setsockopt(socket.IPPROTO_IP, ip_recvdstaddr, 1)
    s.bind(("0.0.0.0", a.port))
    s.settimeout(0.5)
    end = time.time() + a.seconds
    n = 0
    t0 = time.time()
    while time.time() < end:
        try:
            data, anc, _flags, src = s.recvmsg(65535, 256)
        except socket.timeout:
            continue
        n += 1
        dst = [socket.inet_ntoa(c[2][:4]) for c in anc if c[1] == ip_recvdstaddr]
        print(f"[{time.time() - t0:7.3f}s] #{n} from {src[0]}:{src[1]} to {dst} len={len(data)} repr={data!r}")
        print(hexdump(data), flush=True)
    print(f"done, {n} datagrams", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
