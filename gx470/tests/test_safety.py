"""The allowlist must refuse anything that is not a read."""
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from opendiag_link import BlockedCommand, check_allowed  # noqa: E402

ALLOWED = ["ATZ", "AT E0", "ATSP0", "ATSH7E0", "ATST19", "ATRV", "ATDPN",
           "0100", "010C", "03", "07", "0A", "0902", "2101", "21 83", "1A80", "22F190"]
BLOCKED = ["04",            # clear codes
           "1001", "1002",  # session control (programming session)
           "1101",          # ECU reset
           "14FFFFFF",      # clear diagnostic information
           "2701", "2702",  # security access
           "2E F190 00",    # write data by identifier
           "3B01",          # write data by local identifier
           "3101FF00",      # routine control (erase)
           "34", "36", "37",  # download / transfer / exit
           "3D", "3E00", "85",
           "AT PROGV D 00004E20",  # 20 V on pin 13
           "AT VIF PASSTHRU", "AT VIF SLCAN", "ATMA", "ATPP FF ON",
           "AT CV 1200", "hello", "0"]


def test():
    for c in ALLOWED:
        check_allowed(c)
    for c in BLOCKED:
        try:
            check_allowed(c)
        except BlockedCommand:
            continue
        raise AssertionError(f"not blocked: {c}")
    print(f"ok: {len(ALLOWED)} allowed, {len(BLOCKED)} blocked")


if __name__ == "__main__":
    test()
