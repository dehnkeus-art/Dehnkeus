# 2006 Lexus GX 470: read-only OpenDIAG toolkit (macOS)

Goal: understand why the transmission hunts between 4th and 5th when towing
on cruise control, starting by learning what the truck's ECM will tell us.

**Everything here is read-only.** `opendiag_link.py` refuses any request that
isn't a read (no clearing codes, no security access, no writes, no flashing,
no programming voltage) before it reaches the adapter. `tests/test_safety.py`
checks that list.

| Script | When | What it does |
| --- | --- | --- |
| `scan.py` | Parked, ignition ON (engine off or idling) | Bus type, VIN, calibration ID, ECUs, supported data, trouble codes, Toyota `21xx` data sweep |
| `log_live.py` | Test drive, run by a passenger | Logs RPM, speed, throttle, load ... to CSV; Enter marks a gear hunt |
| `sniff_can.py` | Parked or driving, only if `scan.py` says CAN | Listen-only raw CAN recording (never transmits) |

Each run makes a folder in `captures/` with the results plus `transcript.txt`
(every byte sent and received).

## One-time setup on the Mac

1. Open **Terminal** and get this folder:
   ```sh
   git clone -b claude/lexus-gx470-controller-flash-co8923 https://github.com/dehnkeus-art/Dehnkeus.git
   cd Dehnkeus/gx470
   ```
2. Python environment (if macOS offers to install the command-line developer tools, accept):
   ```sh
   python3 -m venv .venv
   source .venv/bin/activate
   pip install -r requirements.txt
   python3 tests/test_safety.py      # should print "ok: ..."
   ```
   In later Terminal windows, run `cd Dehnkeus/gx470 && source .venv/bin/activate` first.

## At the truck

1. Park, set the parking brake. If the session will be long with the engine
   off, connect a battery charger/maintainer.
2. Plug the OpenDIAG into the OBD port (under the dash, left of the steering
   column) and into the Mac by USB.
3. Ignition **ON** (engine can be off).
4. Run the scan:
   ```sh
   python3 scan.py
   ```
   The scripts find the adapter by themselves. If they can't, find the port with
   ```sh
   ioreg -r -n OpenDiag -l -w 0 | grep -E 'bInterfaceNumber|IOCalloutDevice'
   ```
   and use the `/dev/cu.usbmodem...` listed under `"bInterfaceNumber" = 1`:
   `python3 scan.py --port /dev/cu.usbmodem12301`
5. If the scan says **CAN**, also record a minute of raw traffic with the engine idling:
   ```sh
   python3 sniff_can.py --seconds 60
   ```

## The towing test drive

The **passenger** runs the laptop; the driver drives. Start logging while
still parked:

```sh
python3 log_live.py
```

Tow on cruise until it hunts. Each time it shifts 4→5→4, the passenger presses
**Enter**. Press **Ctrl-C** when done. (OpenDIAG's manual says not to operate
the adapter while driving. That's why the passenger runs it, and nothing is
sent to the truck except read requests.)

If `scan.py` found Toyota data on CAN, add those identifiers to the log, e.g.
`python3 log_live.py --raw 7E0:2101 --raw 7E0:2102`.

## Getting the results back to Claude

Commit and push the `captures/` folder to this branch:

```sh
git add captures && git commit -m "GX470 captures" && git push
```

Or attach a zip of `captures/` in the chat. The captures include the VIN.

## Testing without the truck

`tests/fake_opendiag.py` pretends to be an OpenDIAG on a CAN truck or a K-line
truck (Linux/macOS):

```sh
python3 tests/fake_opendiag.py can     # prints a port path such as /dev/ttys004
python3 scan.py --port /dev/ttys004    # in another Terminal window
```

## Adapter source

OpenDIAG by maximus64: [hardware](https://github.com/maximus64/opendiag-hardware),
[firmware and manual](https://github.com/maximus64/opendiag-firmware).
