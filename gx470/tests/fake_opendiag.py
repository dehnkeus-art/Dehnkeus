#!/usr/bin/env python3
"""Fake OpenDIAG on a pseudo-terminal, for testing the scripts without a truck.

  python3 tests/fake_opendiag.py can     # prints the port path, then serves
  python3 tests/fake_opendiag.py kline

It also records every request it receives to fake_requests.log so tests can
prove nothing outside the read-only allowlist was ever sent.
"""

import os
import random
import select
import sys
import time
import tty

MODE = sys.argv[1] if len(sys.argv) > 1 else "can"
LOG = open(os.environ.get("FAKE_LOG", "fake_requests.log"), "w", buffering=1)

VIN = b"JTJBT20X960000000"  # fake VIN, correct length


def can_reply(cmd: str, header: str) -> list[str]:
    def sf(src, *b):
        return f"{src} {len(b):02X} " + " ".join(f"{x:02X}" for x in b)

    def mf(src, payload: bytes):
        out = [f"{src} 1{len(payload) >> 8:X} {len(payload) & 0xFF:02X} " +
               " ".join(f"{x:02X}" for x in payload[:6])]
        rest, n = payload[6:], 1
        while rest:
            out.append(f"{src} 2{n & 0xF:X} " + " ".join(f"{x:02X}" for x in rest[:7]))
            rest, n = rest[7:], n + 1
        return out

    if header in ("7DF", "7E0"):
        if cmd == "0100":
            return [sf("7E8", 0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x11)]
        if cmd == "0120":
            return [sf("7E8", 0x41, 0x20, 0x80, 0x00, 0x00, 0x00)]
        if cmd == "0900":
            return [sf("7E8", 0x49, 0x00, 0x55, 0x40, 0x00, 0x00)]
        if cmd == "090C" or cmd == "0902":
            return mf("7E8", bytes([0x49, 0x02, 0x01]) + VIN)
        if cmd == "0904":
            return mf("7E8", bytes([0x49, 0x04, 0x01]) + b"34811000\0\0\0\0\0\0\0\0")
        if cmd == "03":
            return [sf("7E8", 0x43, 0x01, 0x07, 0x71)]  # P0771
        if cmd in ("07", "0A"):
            return [sf("7E8", int(cmd, 16) + 0x40, 0x00)]
        if cmd == "010C":
            rpm = random.randint(1700, 2900) * 4
            return [sf("7E8", 0x41, 0x0C, rpm >> 8, rpm & 0xFF)]
        if cmd == "010D":
            return [sf("7E8", 0x41, 0x0D, 100)]
        if cmd.startswith("01") and len(cmd) == 4:
            return [sf("7E8", 0x41, int(cmd[2:], 16), 0x55)]
    if header == "7E0" and cmd in ("2101", "2102", "2183"):
        return mf("7E8", bytes([0x61, int(cmd[2:], 16)]) + bytes(range(10)))
    if header == "7E0" and cmd.startswith("21"):
        return [sf("7E8", 0x7F, 0x21, 0x12)]
    return ["NO DATA"]


def kline_reply(cmd: str) -> list[str]:
    def fr(*b):
        b = (0x48, 0x6B, 0x10) + b
        return " ".join(f"{x:02X}" for x in b + (sum(b) & 0xFF,))

    if cmd == "0100":
        return [fr(0x41, 0x00, 0xBE, 0x3E, 0xB8, 0x10)]
    if cmd == "0900":
        return [fr(0x49, 0x00, 0x01, 0x54, 0x00, 0x00, 0x00)]
    if cmd == "0902":
        chunks = [b"\0\0\0" + VIN[:1]] + [VIN[i:i + 4] for i in range(1, 17, 4)]
        return [fr(0x49, 0x02, i + 1, *c) for i, c in enumerate(chunks)]
    if cmd == "03":
        return [fr(0x43, 0x07, 0x71, 0x01, 0x10, 0, 0),
                fr(0x43, 0x03, 0x00, 0, 0, 0, 0)]  # P0771 P0110 P0300
    if cmd in ("07", "0A"):
        return [fr(int(cmd, 16) + 0x40, 0, 0, 0, 0, 0, 0)]
    if cmd == "010C":
        return [fr(0x41, 0x0C, 0x1F, 0x40)]
    if cmd.startswith("01") and len(cmd) == 4:
        return [fr(0x41, int(cmd[2:], 16), 0x55)]
    return ["NO DATA"]


def main():
    master, slave = os.openpty()
    tty.setraw(slave)
    print(os.ttyname(slave), flush=True)
    header, slcan, open_ = "7DF", False, False
    buf = b""
    while True:
        ready, _, _ = select.select([master], [], [], 0.01)
        if slcan and open_:
            os.write(master, b"t2C48" + os.urandom(8).hex().upper().encode() + b"\r")
            os.write(master, b"t3B06" + os.urandom(6).hex().upper().encode() + b"\r")
        if not ready:
            continue
        data = os.read(master, 1024)
        if not data:
            continue
        buf += data
        while b"\r" in buf:
            line, buf = buf.split(b"\r", 1)
            cmd = line.decode().strip().upper().replace(" ", "")
            LOG.write(cmd + "\n")
            if slcan:
                if cmd == "L":
                    open_ = True
                    os.write(master, b"\r")
                    continue
                if cmd == "C":
                    open_ = False
                os.write(master, b"\r")
                continue
            if cmd == "ATVIFSLCAN":
                slcan = True
                os.write(master, b"OK\r\r>")
                continue
            if cmd.startswith("AT"):
                a = cmd[2:]
                if a.startswith("SH"):
                    header = a[2:]
                if a == "Z":
                    time.sleep(0.2)
                    out = "ELM327 v2.3"
                elif a == "I":
                    out = "ELM327 v2.3"
                elif a == "@1":
                    out = "OpenDiag fake"
                elif a == "RV":
                    out = "12.6V"
                elif a == "DPN":
                    out = "A6" if MODE == "can" else "A3"
                elif a == "DP":
                    out = ("AUTO, ISO 15765-4 (CAN 11/500)" if MODE == "can"
                           else "AUTO, ISO 9141-2")
                else:
                    out = "OK"
                os.write(master, (out + "\r\r>").encode())
                continue
            lines = can_reply(cmd, header) if MODE == "can" else kline_reply(cmd)
            os.write(master, ("\r".join(lines) + "\r\r>").encode())


if __name__ == "__main__":
    main()
