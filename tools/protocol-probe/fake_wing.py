#!/usr/bin/env python3
"""Minimal fake WING used to observe what WING-Edit does after discovery (no root needed).

- UDP 2222: answers 'WING?' with 'WING,<ip>,<name>,<model>,<serial>,<fw>' (sent from port 2222
  back to the requester's ip:port), logs every datagram.
- UDP 2223: logs OSC datagrams; answers '/?' with the same info string as an OSC ',s' message.
- TCP 2222: accepts connections and hexdumps everything received (decoded per NRP channel too).
  --tcp-mode silent : never send anything (observe first bytes / timeout behaviour)
  --tcp-mode end    : reply 0xDE (end of data/def request) on the same channel to each 0xDC/0xDD
  --tcp-mode replay : reply using a JSONL file of {"req": hex, "resp": hex} pairs (exact match on the
                      decoded request token sequence), else fallback to 'end'.

Usage: python3 fake_wing.py --seconds 60 [--ip 192.168.20.130] [--fw 3.0.5-0-g0c2b9d4a:release]
Everything is logged to stdout (pipe through tee).
"""
import argparse
import select
import socket
import struct
import sys
import time

T0 = time.time()


def ts() -> str:
    return f"[{time.time() - T0:8.3f}s]"


def hexdump(data: bytes, indent: str = "    ") -> str:
    out = []
    for off in range(0, len(data), 16):
        chunk = data[off:off + 16]
        hx = " ".join(f"{b:02x}" for b in chunk)
        asc = "".join(chr(b) if 32 <= b < 127 else "." for b in chunk)
        out.append(f"{indent}{off:08x}  {hx:<47}  |{asc}|")
    return "\n".join(out)


def osc_str(s: str) -> bytes:
    b = s.encode() + b"\x00"
    return b + b"\x00" * ((4 - len(b) % 4) % 4)


class NrpRx:
    """Native-protocol receive state machine (per Behringer sample routine)."""

    def __init__(self):
        self.esc = False
        self.ch = -1

    def feed(self, data: bytes):
        """Yield (channel, byte) for each unescaped payload byte."""
        for db in data:
            if db == 0xDF and not self.esc:
                self.esc = True
                continue
            if self.esc:
                if db != 0xDF:
                    self.esc = False
                    if db == 0xDE:
                        db = 0xDF
                    elif 0xD0 <= db < 0xD0 + 14:
                        self.ch = db - 0xD0
                        continue
                    elif self.ch >= 0:
                        yield (self.ch, 0xDF)
                # db == 0xDF while esc: literal 0xDF then stay escaped? (sample code: falls through)
            if self.ch >= 0:
                yield (self.ch, db)


def nrp_escape(payload: bytes) -> bytes:
    out = bytearray()
    for b in payload:
        out.append(b)
        if b == 0xDF:
            out.append(0xDE)
    return bytes(out)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=60)
    ap.add_argument("--ip", default="192.168.20.130")
    ap.add_argument("--name", default="WINGSIM")
    ap.add_argument("--model", default="ngc-full")
    ap.add_argument("--serial", default="S0000000000SIM")
    ap.add_argument("--fw", default="3.3.4-0-g0c2b9d4a:release")
    ap.add_argument("--tcp-mode", default="silent", choices=["silent", "end"])
    ap.add_argument("--no-tcp", action="store_true")
    a = ap.parse_args()

    info = f"WING,{a.ip},{a.name},{a.model},{a.serial},{a.fw}"
    print(f"{ts()} discovery reply = {info!r}", flush=True)

    def udp(port):
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        s.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)
        s.bind(("0.0.0.0", port))
        return s

    u2222 = udp(2222)
    u2223 = udp(2223)
    socks = [u2222, u2223]
    lst = None
    if not a.no_tcp:
        lst = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        lst.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        lst.bind(("0.0.0.0", 2222))
        lst.listen(16)
        socks.append(lst)
    conns = {}  # sock -> (addr, NrpRx, pending tokens)
    end = time.time() + a.seconds
    disc_count = 0
    while time.time() < end:
        r, _, _ = select.select(socks + list(conns), [], [], 0.25)
        for s in r:
            if s is u2222:
                data, src = s.recvfrom(65535)
                disc_count += 1
                if data == b"WING?":
                    if disc_count <= 4 or disc_count % 20 == 0:
                        print(f"{ts()} UDP2222 <- {src} {data!r} (#{disc_count}); replying", flush=True)
                    s.sendto(info.encode(), src)
                else:
                    print(f"{ts()} UDP2222 <- {src} len={len(data)}\n{hexdump(data)}", flush=True)
            elif s is u2223:
                data, src = s.recvfrom(65535)
                print(f"{ts()} UDP2223(OSC) <- {src} len={len(data)}\n{hexdump(data)}", flush=True)
                if data.startswith(b"/?\x00"):
                    s.sendto(osc_str("/?") + osc_str(",s") + osc_str(info), src)
            elif s is lst:
                c, addr = lst.accept()
                c.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f"{ts()} TCP2222 accept from {addr}", flush=True)
                conns[c] = [addr, NrpRx()]
            else:
                addr, rx = conns[s]
                try:
                    data = s.recv(65536)
                except OSError as e:
                    data = b""
                    print(f"{ts()} TCP {addr} error {e}", flush=True)
                if not data:
                    print(f"{ts()} TCP2222 {addr} closed by peer", flush=True)
                    conns.pop(s)
                    s.close()
                    continue
                print(f"{ts()} TCP2222 <- {addr} len={len(data)}\n{hexdump(data)}", flush=True)
                decoded = list(rx.feed(data))
                if decoded:
                    bych = {}
                    for ch, b in decoded:
                        bych.setdefault(ch, bytearray()).append(b)
                    for ch, bs in bych.items():
                        print(f"    decoded ch{ch}: {bs.hex()}", flush=True)
                if a.tcp_mode == "end":
                    out = bytearray()
                    for ch, b in decoded:
                        if b in (0xDC, 0xDD):
                            out += bytes([0xDF, 0xD0 + ch, 0xDE])
                    if out:
                        print(f"{ts()} TCP2222 -> {addr} {out.hex()}", flush=True)
                        s.sendall(out)
    print(f"{ts()} done; discovery probes seen={disc_count}; open conns={len(conns)}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
