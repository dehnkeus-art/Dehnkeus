#!/usr/bin/env python3
"""Read-only identification scan of the truck.

Ignition ON, engine OFF (or idling), parked. Takes a few minutes.

Collects: adapter info, battery voltage, OBD protocol, responding ECUs,
supported PIDs, VIN / calibration IDs, stored / pending / permanent codes,
one snapshot of every supported live PID, and (on CAN only) a sweep of the
Toyota read-only "21 xx" local identifiers on the engine/transmission
addresses. Nothing is written to the vehicle.
"""

import argparse
import sys

from opendiag_link import decode_dtcs, decode_pid, open_link


def supported(link, mode: int) -> dict[str, list[int]]:
    """Walk the supported-PID bitmaps (00, 20, 40 ...) for every ECU."""
    result: dict[str, list[int]] = {}
    base = 0x00
    while base <= 0xE0:
        r = link.request(f"{mode:02X}{base:02X}")
        if not r.ok:
            break
        more = False
        for ecu, p in r.by_ecu.items():
            if len(p) < 6 or p[0] != mode + 0x40:
                continue
            bits = int.from_bytes(p[2:6], "big")
            for i in range(32):
                if bits & (1 << (31 - i)):
                    result.setdefault(ecu, []).append(base + i + 1)
            if bits & 1:
                more = True
        if not more:
            break
        base += 0x20
    return result


def ascii_of(b: bytes) -> str:
    return "".join(chr(c) for c in b if 32 <= c < 127).strip()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--port", help="data port, e.g. /dev/cu.usbmodem12301")
    ap.add_argument("--protocol", default="0",
                    help="ELM327 protocol number (0 = auto, the default)")
    ap.add_argument("--no-sweep", action="store_true",
                    help="skip the Toyota 21xx local-identifier sweep")
    args = ap.parse_args()

    link = open_link(args.port, "scan")
    s = link.session
    report: dict = {}
    try:
        print("Adapter ...")
        report["adapter"] = link.init()
        print(f"  {report['adapter']}")
        v = report["adapter"]["battery_v"].rstrip("Vv ")
        try:
            if float(v) < 12.0:
                print(f"  WARNING: battery reads {v} V. Connect a charger/maintainer.")
        except ValueError:
            pass

        print("Connecting to vehicle (protocol search can take ~15 s) ...")
        report["bus"] = link.connect_vehicle(args.protocol)
        print(f"  {report['bus']['protocol']}  can={link.is_can}")
        if not report["bus"]["connected"]:
            print("  No OBD reply. Is the ignition ON? See transcript.txt.")
            s.write_json("report.json", report)
            return 1

        print("Supported PIDs ...")
        report["mode01_supported"] = {e: [f"{p:02X}" for p in v]
                                      for e, v in supported(link, 0x01).items()}
        report["mode09_supported"] = {e: [f"{p:02X}" for p in v]
                                      for e, v in supported(link, 0x09).items()}
        print(f"  ECUs answering OBD: {sorted(report['mode01_supported'])}")

        print("Vehicle info (mode 09) ...")
        info = {}
        for pid, name in ((0x02, "vin"), (0x04, "calibration_id"),
                          (0x06, "cvn"), (0x0A, "ecu_name")):
            r = link.request(f"09{pid:02X}", timeout=8)
            for ecu, p in r.by_ecu.items():
                if link.is_can:
                    raw = p[2:]
                else:
                    # Legacy: each frame is 49 pid index d d d d, in order.
                    fr = sorted(r.frames.get(ecu, []), key=lambda f: f[2] if len(f) > 2 else 0)
                    raw = b"".join(f[3:] for f in fr)
                    p = b"".join(fr)
                if name == "cvn":
                    val = raw.hex(" ")
                else:
                    # CAN replies have a 1-byte count; legacy has per-frame indexes.
                    val = ascii_of(raw)
                info.setdefault(ecu, {})[name] = val
                info[ecu][name + "_raw"] = p.hex(" ")
        report["vehicle_info"] = info
        for ecu, d in info.items():
            print(f"  {ecu}: VIN={d.get('vin', '?')}  CALID={d.get('calibration_id', '?')}")

        print("Trouble codes ...")
        dtcs = {}
        for mode, name in ((0x03, "stored"), (0x07, "pending"), (0x0A, "permanent")):
            r = link.request(f"{mode:02X}")
            for ecu, p in r.by_ecu.items():
                if p and p[0] == mode + 0x40:
                    legacy = None if link.is_can else r.frames.get(ecu)
                    dtcs.setdefault(ecu, {})[name] = decode_dtcs(p, legacy)
        report["dtcs"] = dtcs
        for ecu, d in dtcs.items():
            print(f"  {ecu}: {d}")

        print("Live data snapshot ...")
        snap = {}
        for ecu, pids in report["mode01_supported"].items():
            for pid_hex in pids:
                pid = int(pid_hex, 16)
                if pid % 0x20 == 0:  # bitmap PIDs
                    continue
                r = link.request(f"01{pid:02X}")
                p = r.by_ecu.get(ecu)
                if p:
                    d = decode_pid(pid, p)
                    if d:
                        snap.setdefault(ecu, {})[d[0]] = d[1]
        report["live_snapshot"] = snap

        if link.is_can and not args.no_sweep:
            print("Toyota local-identifier sweep (21 01..FF) ...")
            link.at("ST19")  # ~100 ms timeout per request
            sweep = {}
            for addr in ("7E0", "7E1", "7E2"):
                link.set_header(addr)
                probe = link.request("2101")
                if not probe.ok:
                    print(f"  {addr}: no reply to 2101, skipping")
                    continue
                for lid in range(0x01, 0x100):
                    r = link.request(f"21{lid:02X}")
                    for ecu, p in r.by_ecu.items():
                        if p and p[0] == 0x61:
                            sweep.setdefault(addr, {})[f"{lid:02X}"] = {
                                "from": ecu, "data": p.hex(" ")}
                print(f"  {addr}: {len(sweep.get(addr, {}))} identifiers answered")
            report["toyota_21_sweep"] = sweep
            link.at("ST32")
            link.set_header("7DF")
        elif not link.is_can:
            report["toyota_21_sweep"] = "skipped: not a CAN bus"
            print("Not a CAN bus: skipping the Toyota sweep (needs K-line addressing).")

        s.write_json("report.json", report)
        print(f"\nDone. Results: {s.path('report.json')}")
        return 0
    except KeyboardInterrupt:
        print("\nStopped. Partial results saved.")
        s.write_json("report.json", report)
        return 130
    finally:
        link.close()


if __name__ == "__main__":
    sys.exit(main())
