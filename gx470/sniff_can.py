#!/usr/bin/env python3
"""Passively record raw CAN traffic on the OBD port (LISTEN-ONLY).

The adapter is switched to SLCAN and the channel is opened with `L`
(listen-only): it never transmits, never acknowledges, and refuses any
send. Only useful if scan.py reported a CAN bus.

Note: many Toyotas of this era put a gateway behind the OBD port, so the
port may be silent unless a tool is asking questions. Silence is a result.

Output: candump.log (candump -l format, opens in SavvyCAN / python-can),
ids.txt (every ID seen, count, rate), markers.txt (Enter presses).
"""

import argparse
import sys
import threading
import time

from opendiag_link import open_link

RATES = {"500k": "S6", "250k": "S5", "125k": "S4"}


def slcan_cmd(ser, cmd: str) -> bytes:
    ser.write((cmd + "\r").encode())
    ser.flush()
    time.sleep(0.1)
    return ser.read(64)


def parse(line: str):
    """Return (id_hex, data_hex) for t/T frames, else None."""
    try:
        if line.startswith("t"):
            ident, n = line[1:4], int(line[4], 16)
            return ident, line[5:5 + 2 * n]
        if line.startswith("T"):
            ident, n = line[1:9], int(line[9], 16)
            return ident, line[10:10 + 2 * n]
    except (ValueError, IndexError):
        pass
    return None


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--rate", choices=["auto", *RATES], default="auto")
    ap.add_argument("--seconds", type=float, default=0, help="stop after N s (0 = Ctrl-C)")
    args = ap.parse_args()

    link = open_link(args.port, "sniff")
    s, ser = link.session, link.ser
    counts: dict[str, int] = {}
    markers = []
    try:
        link.enter_slcan()
        slcan_cmd(ser, "C")
        chosen = None
        for name in ([args.rate] if args.rate != "auto" else list(RATES)):
            slcan_cmd(ser, RATES[name])
            slcan_cmd(ser, "L")  # listen-only
            ser.reset_input_buffer()
            t_end, seen = time.time() + 3, 0
            while time.time() < t_end:
                seen += ser.read(4096).count(b"\r")
            print(f"  {name}: {seen} frames in 3 s")
            if seen > 5 or args.rate != "auto":
                chosen = name
                break
            slcan_cmd(ser, "C")
        if not chosen:
            print("No CAN traffic heard at any rate. The OBD port is probably gatewayed.")
            return 0

        print(f"Recording at {chosen}, listen-only. Enter = marker, Ctrl-C = stop.")

        def wait_enter():
            for _ in sys.stdin:
                markers.append(time.time())
                print(f"  >> marker {len(markers)}")
        threading.Thread(target=wait_enter, daemon=True).start()

        buf = b""
        t_stop = time.time() + args.seconds if args.seconds else None
        with open(s.path("candump.log"), "w") as out:
            last = 0.0
            while t_stop is None or time.time() < t_stop:
                buf += ser.read(4096)
                *lines, buf = buf.split(b"\r")
                now = time.time()
                for raw in lines:
                    fr = parse(raw.decode("ascii", "replace").strip())
                    if fr:
                        out.write(f"({now:.6f}) can0 {fr[0]}#{fr[1]}\n")
                        counts[fr[0]] = counts.get(fr[0], 0) + 1
                if now - last > 2:
                    last = now
                    print(f"  {sum(counts.values())} frames, {len(counts)} IDs")
    except KeyboardInterrupt:
        pass
    finally:
        try:
            slcan_cmd(ser, "C")
        except Exception:
            pass
        dur = max(time.time() - s.t0, 1e-6)
        with open(s.path("ids.txt"), "w") as f:
            for ident, n in sorted(counts.items()):
                f.write(f"{ident}\t{n}\t{n / dur:.1f}/s\n")
        with open(s.path("markers.txt"), "w") as f:
            f.writelines(f"{t:.6f}\n" for t in markers)
        print(f"Saved to {s.dir}")
        link.close()  # closing the port returns the adapter to ELM327 mode


if __name__ == "__main__":
    sys.exit(main())
