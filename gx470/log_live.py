#!/usr/bin/env python3
"""Log live data to CSV while the truck is driven (read-only).

A PASSENGER runs this. Start it parked, before moving. Whenever the
transmission hunts between 4th and 5th, the passenger presses Enter: the
row gets a marker so the hunts are easy to find afterwards.

Examples:
  python3 log_live.py
  python3 log_live.py --pids 0C,0D,04,11,05,0B
  python3 log_live.py --raw 7E0:2101 --raw 7E0:2102   # Toyota identifiers from scan.py
"""

import argparse
import csv
import sys
import threading
import time

from opendiag_link import PIDS, decode_pid, open_link

DEFAULT_PIDS = "0C,0D,04,11,05,0B,0E"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port")
    ap.add_argument("--protocol", default="0")
    ap.add_argument("--pids", default=DEFAULT_PIDS, help="mode 01 PIDs, hex, comma separated")
    ap.add_argument("--raw", action="append", default=[],
                    help="extra read request as HEADER:HEX (CAN only), e.g. 7E0:2101")
    args = ap.parse_args()

    pids = [int(p, 16) for p in args.pids.split(",") if p.strip()]
    raws = [r.split(":", 1) for r in args.raw]

    link = open_link(args.port, "drive")
    s = link.session
    markers = {"n": 0}

    def wait_enter():
        for _ in sys.stdin:
            markers["n"] += 1
            print(f"  >> marker {markers['n']} at {time.time() - s.t0:.1f}s")

    try:
        link.init()
        bus = link.connect_vehicle(args.protocol)
        if not bus["connected"]:
            sys.exit("No OBD reply. Ignition ON? See transcript.txt in " + s.dir)
        if raws and not link.is_can:
            print("Not a CAN bus: --raw requests ignored.")
            raws = []
        if link.is_can:
            link.at("ST19")  # 100 ms per request keeps the sample rate up
            link.set_header("7DF")

        names = [PIDS.get(p, (f"pid_{p:02X}",))[0] for p in pids]
        cols = ["t_s", "marker"] + names + (["speed_mph"] if 0x0D in pids else []) \
            + [f"raw_{h}_{c}" for h, c in raws]

        with open(s.path("drive.csv"), "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(cols)
            threading.Thread(target=wait_enter, daemon=True).start()
            print("Logging. Press Enter to mark a gear hunt, Ctrl-C to stop.")
            last_print = 0.0
            while True:
                row = {"t_s": round(time.time() - s.t0, 3), "marker": markers["n"]}
                for pid in pids:
                    r = link.request(f"01{pid:02X}")
                    for p in r.by_ecu.values():
                        d = decode_pid(pid, p)
                        if d:
                            row[d[0]] = d[1]
                            break
                if "speed_kph" in row:
                    row["speed_mph"] = round(row["speed_kph"] * 0.621371, 1)
                for h, c in raws:
                    link.set_header(h)
                    r = link.request(c)
                    row[f"raw_{h}_{c}"] = " ".join(p.hex(" ") for p in r.by_ecu.values())
                if raws:
                    link.set_header("7DF")
                w.writerow([row.get(c, "") for c in cols])
                f.flush()
                if time.time() - last_print > 1:
                    last_print = time.time()
                    print("  " + "  ".join(f"{k}={row[k]}" for k in
                                           ("rpm", "speed_mph", "throttle_pct", "engine_load_pct")
                                           if k in row))
    except KeyboardInterrupt:
        print(f"\nStopped. Log: {s.path('drive.csv')}")
    finally:
        link.close()


if __name__ == "__main__":
    main()
