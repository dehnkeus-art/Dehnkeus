"""Read-only link to an OpenDIAG adapter (ELM327 mode) on macOS/Linux.

Every request goes through a safety allowlist: only read services are sent to
the vehicle. Clearing codes, security access, routines, writes, programming
and the adapter's programming-voltage commands are refused before they reach
the serial port.

Every byte sent and received is written to a transcript so a session can be
analysed later without the truck.
"""

from __future__ import annotations

import glob
import json
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass, field

import serial  # pyserial

# --------------------------------------------------------------------------
# Safety allowlist
# --------------------------------------------------------------------------

# AT commands that only configure the ELM327 layer or read adapter state.
# Compared after removing "AT" and spaces, uppercased.
AT_ALLOWED = re.compile(
    r"^(Z|WS|D|I|@1|RV|DP|DPN|E[01]|L[01]|S[01]|H[01]|M[01]|AL|NL|IA|PC|KW|"
    r"CAF[01]|AT[012]|SP[0-9A-C]|SPA[0-9A-C]|TP[0-9A-C]|TPA[0-9A-C]|"
    r"ST[0-9A-F]{2}|SH[0-9A-F]{3}|SH[0-9A-F]{6}|SH[0-9A-F]{8})$"
)

# Diagnostic services that only read.
#   01 current data, 02 freeze frame, 03 stored DTCs, 07 pending DTCs,
#   09 vehicle info, 0A permanent DTCs,
#   1A KWP ReadEcuIdentification, 21 KWP ReadDataByLocalIdentifier,
#   22 ReadDataByIdentifier.
READ_SERVICES = {0x01, 0x02, 0x03, 0x07, 0x09, 0x0A, 0x1A, 0x21, 0x22}


class BlockedCommand(Exception):
    pass


def check_allowed(cmd: str) -> None:
    c = cmd.strip().upper().replace(" ", "")
    if c.startswith("AT"):
        if not AT_ALLOWED.match(c[2:]):
            raise BlockedCommand(f"AT command not on the read-only allowlist: {cmd!r}")
        return
    if not re.fullmatch(r"[0-9A-F]{2,}", c) or len(c) % 2:
        raise BlockedCommand(f"not a hex request: {cmd!r}")
    service = int(c[:2], 16)
    if service not in READ_SERVICES:
        raise BlockedCommand(f"service 0x{service:02X} is not a read service: {cmd!r}")


# --------------------------------------------------------------------------
# Port discovery (macOS: ioreg; Linux: /dev/serial/by-id)
# --------------------------------------------------------------------------

def find_data_port() -> str | None:
    if sys.platform == "darwin":
        try:
            out = subprocess.run(
                ["ioreg", "-r", "-n", "OpenDiag", "-l", "-w", "0"],
                capture_output=True, text=True, timeout=10,
            ).stdout
        except Exception:
            out = ""
        iface = None
        for line in out.splitlines():
            m = re.search(r'"bInterfaceNumber" = (\d+)', line)
            if m:
                iface = int(m.group(1))
            m = re.search(r'"IOCalloutDevice" = "([^"]+)"', line)
            if m and iface == 1:  # interface 1 = data link, 3 = debug console
                return m.group(1)
        return None
    for p in sorted(glob.glob("/dev/serial/by-id/*OpenD*-if00")):
        return os.path.realpath(p)
    return None


# --------------------------------------------------------------------------
# Session output
# --------------------------------------------------------------------------

class Session:
    """A timestamped capture folder with a raw transcript."""

    def __init__(self, kind: str, root: str = "captures"):
        stamp = time.strftime("%Y%m%d-%H%M%S")
        self.dir = os.path.join(root, f"{stamp}-{kind}")
        os.makedirs(self.dir, exist_ok=True)
        self.t0 = time.time()
        self._tx = open(os.path.join(self.dir, "transcript.txt"), "a", buffering=1)

    def log(self, direction: str, text: str) -> None:
        t = time.time() - self.t0
        for line in text.replace("\r", "\n").split("\n"):
            if line.strip():
                self._tx.write(f"{t:10.3f} {direction} {line}\n")

    def path(self, name: str) -> str:
        return os.path.join(self.dir, name)

    def write_json(self, name: str, obj) -> None:
        with open(self.path(name), "w") as f:
            json.dump(obj, f, indent=2, sort_keys=False)

    def close(self) -> None:
        self._tx.close()


# --------------------------------------------------------------------------
# ELM327 link
# --------------------------------------------------------------------------

NOISE = ("SEARCHING", "BUS INIT", "STOPPED")
FAILURES = ("NO DATA", "UNABLE TO CONNECT", "CAN ERROR", "BUS ERROR",
            "BUFFER FULL", "FB ERROR", "DATA ERROR", "ERROR", "?", "ACT ALERT",
            "LV RESET", "LP ALERT")


@dataclass
class Reply:
    cmd: str
    lines: list[str]
    ok: bool
    # source address -> reassembled payload bytes (service byte first)
    by_ecu: dict[str, bytes] = field(default_factory=dict)
    # source address -> list of raw frames (legacy buses: one per message)
    frames: dict[str, list[bytes]] = field(default_factory=dict)


class ElmLink:
    def __init__(self, port: str, session: Session, timeout: float = 5.0):
        self.session = session
        self.ser = serial.Serial(port, 115200, timeout=0.05)
        self.default_timeout = timeout
        self.is_can = False
        self.header = None

    # -- raw I/O ------------------------------------------------------------

    def _write(self, text: str) -> None:
        self.session.log(">>", text)
        self.ser.write((text + "\r").encode("ascii"))
        self.ser.flush()

    def _read_until_prompt(self, timeout: float) -> str:
        buf = bytearray()
        deadline = time.time() + timeout
        while time.time() < deadline:
            chunk = self.ser.read(256)
            if chunk:
                buf += chunk
                if buf.rstrip().endswith(b">"):
                    break
        text = buf.decode("ascii", "replace")
        self.session.log("<<", text.rstrip(">").rstrip())
        return text

    def raw(self, cmd: str, timeout: float | None = None) -> list[str]:
        check_allowed(cmd)
        self.ser.reset_input_buffer()
        self._write(cmd)
        text = self._read_until_prompt(timeout or self.default_timeout)
        lines = []
        for line in text.replace("\r", "\n").split("\n"):
            line = line.strip().lstrip(">").strip()
            if line and line.upper().replace(" ", "") != cmd.upper().replace(" ", ""):
                lines.append(line)
        return lines

    # -- setup --------------------------------------------------------------

    def at(self, cmd: str, timeout: float | None = None) -> str:
        lines = self.raw("AT" + cmd, timeout)
        return " / ".join(lines)

    def init(self) -> dict:
        info = {}
        self.at("Z", timeout=3)
        time.sleep(0.5)
        for c in ("E0", "L0", "S1", "H1", "AL", "CAF1"):
            self.at(c)
        info["adapter"] = self.at("I")
        info["adapter_desc"] = self.at("@1")
        info["battery_v"] = self.at("RV")
        return info

    def connect_vehicle(self, protocol: str = "0") -> dict:
        self.at("SP" + protocol)
        self.request("0100", timeout=20)  # triggers the protocol search
        dpn = self.at("DPN").upper().lstrip("A")
        self.is_can = dpn[:1] in ("6", "7", "8", "9")
        first = self.request("0100", timeout=10)  # parsed with the right framing
        return {
            "protocol_number": dpn,
            "protocol": self.at("DP"),
            "is_can": self.is_can,
            "connected": first.ok,
            "first_reply": first.lines,
        }

    def set_header(self, header: str) -> None:
        if header != self.header:
            self.at("SH" + header)
            self.header = header

    # -- requests -----------------------------------------------------------

    def request(self, hexcmd: str, timeout: float | None = None) -> Reply:
        lines = self.raw(hexcmd, timeout)
        useful = [l for l in lines if not l.upper().startswith(NOISE)]
        failed = not useful or any(l.upper().startswith(FAILURES) for l in useful)
        reply = Reply(hexcmd, useful, ok=not failed)
        if not failed:
            reply.by_ecu, reply.frames = self._assemble(useful)
            reply.ok = bool(reply.by_ecu)
        return reply

    def _assemble(self, lines: list[str]):
        frames: dict[str, list[list[int]]] = {}
        for line in lines:
            toks = line.split()
            try:
                if self.is_can:
                    src = toks[0].upper()
                    data = [int(t, 16) for t in toks[1:]]
                else:
                    # J1850/K-line: 3 header bytes, data, checksum.
                    vals = [int(t, 16) for t in toks]
                    src = f"{vals[2]:02X}"
                    data = vals[3:-1]
            except (ValueError, IndexError):
                continue
            frames.setdefault(src, []).append(data)

        out = {}
        for src, flist in frames.items():
            if self.is_can:
                out[src] = self._isotp(flist)
            else:
                # Legacy buses send one message per frame, each repeating the
                # service byte. by_ecu holds the first; frames holds them all.
                out[src] = bytes(flist[0])
        raw = {k: [bytes(f) for f in v] for k, v in frames.items()}
        return {k: v for k, v in out.items() if v}, raw

    @staticmethod
    def _isotp(flist: list[list[int]]) -> bytes:
        payload = bytearray()
        total = None
        for f in flist:
            if not f:
                continue
            kind = f[0] >> 4
            if kind == 0:  # single frame
                n = f[0] & 0x0F
                return bytes(f[1:1 + n])
            if kind == 1:  # first frame
                total = ((f[0] & 0x0F) << 8) | f[1]
                payload = bytearray(f[2:])
            elif kind == 2:  # consecutive frame
                payload += bytes(f[1:])
        if total is not None:
            return bytes(payload[:total])
        return bytes(payload)

    # -- SLCAN --------------------------------------------------------------

    def enter_slcan(self) -> None:
        """Switch this link to SLCAN. Reverts to ELM327 when the port closes."""
        self.session.log(">>", "AT VIF SLCAN")
        self.ser.reset_input_buffer()
        self.ser.write(b"AT VIF SLCAN\r")
        self.ser.flush()
        time.sleep(0.4)
        self.session.log("<<", self.ser.read(4096).decode("ascii", "replace"))

    def close(self) -> None:
        try:
            self.ser.close()
        finally:
            self.session.close()


def open_link(port: str | None, kind: str) -> ElmLink:
    port = port or find_data_port()
    if not port:
        sys.exit("OpenDIAG data port not found. Plug it in by USB, or pass --port "
                 "/dev/cu.usbmodemXXXX (see README).")
    session = Session(kind)
    print(f"Port: {port}\nSaving to: {session.dir}")
    return ElmLink(port, session)


# --------------------------------------------------------------------------
# Decoding helpers
# --------------------------------------------------------------------------

def decode_dtcs(payload: bytes, legacy_frames: list[bytes] | None = None) -> list[str]:
    """Decode a mode 03/07/0A reply (service byte first).

    CAN: one ISO-TP message with a count byte. Legacy (K-line/J1850): pass
    every frame; each is service byte + three code pairs.
    """
    if legacy_frames:
        body = b"".join(f[1:7] for f in legacy_frames)
    else:
        body = payload[1:]
        if len(body) % 2 == 1:  # CAN count byte
            body = body[1:]
    codes = []
    for i in range(0, len(body) - 1, 2):
        a, b = body[i], body[i + 1]
        if a == 0 and b == 0:
            continue
        letter = "PCBU"[a >> 6]
        codes.append(f"{letter}{(a >> 4) & 3}{a & 0xF:X}{b >> 4:X}{b & 0xF:X}")
    return codes


PIDS = {
    0x04: ("engine_load_pct", lambda d: d[0] * 100 / 255),
    0x05: ("coolant_c", lambda d: d[0] - 40),
    0x06: ("stft_b1_pct", lambda d: d[0] * 100 / 128 - 100),
    0x07: ("ltft_b1_pct", lambda d: d[0] * 100 / 128 - 100),
    0x08: ("stft_b2_pct", lambda d: d[0] * 100 / 128 - 100),
    0x09: ("ltft_b2_pct", lambda d: d[0] * 100 / 128 - 100),
    0x0B: ("map_kpa", lambda d: d[0]),
    0x0C: ("rpm", lambda d: (d[0] * 256 + d[1]) / 4),
    0x0D: ("speed_kph", lambda d: d[0]),
    0x0E: ("timing_adv_deg", lambda d: d[0] / 2 - 64),
    0x0F: ("intake_air_c", lambda d: d[0] - 40),
    0x10: ("maf_gps", lambda d: (d[0] * 256 + d[1]) / 100),
    0x11: ("throttle_pct", lambda d: d[0] * 100 / 255),
    0x1F: ("run_time_s", lambda d: d[0] * 256 + d[1]),
    0x21: ("dist_mil_on_km", lambda d: d[0] * 256 + d[1]),
    0x2F: ("fuel_level_pct", lambda d: d[0] * 100 / 255),
    0x33: ("baro_kpa", lambda d: d[0]),
    0x42: ("module_v", lambda d: (d[0] * 256 + d[1]) / 1000),
    0x43: ("abs_load_pct", lambda d: (d[0] * 256 + d[1]) * 100 / 255),
    0x45: ("rel_throttle_pct", lambda d: d[0] * 100 / 255),
    0x46: ("ambient_c", lambda d: d[0] - 40),
    0x47: ("throttle_b_pct", lambda d: d[0] * 100 / 255),
    0x49: ("pedal_d_pct", lambda d: d[0] * 100 / 255),
    0x4A: ("pedal_e_pct", lambda d: d[0] * 100 / 255),
    0x4C: ("cmd_throttle_pct", lambda d: d[0] * 100 / 255),
}


def decode_pid(pid: int, payload: bytes):
    """payload = full mode-01 reply (41 pid data...). Returns (name, value)."""
    if len(payload) < 3 or payload[0] != 0x41 or payload[1] != pid:
        return None
    name, fn = PIDS.get(pid, (f"pid_{pid:02X}", None))
    data = payload[2:]
    if fn is None:
        return name, data.hex(" ")
    try:
        return name, round(fn(data), 3)
    except IndexError:
        return name, data.hex(" ")
