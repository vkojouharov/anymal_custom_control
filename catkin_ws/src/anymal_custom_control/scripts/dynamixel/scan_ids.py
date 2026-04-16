#!/usr/bin/env python3
"""Scan the Dynamixel bus for responsive motor IDs.

Fastest way to confirm wiring, IDs, and baudrate after any hardware change.
Uses the SDK's broadcast ping — one packet per baud tested — so a full scan
at one baud finishes in ~30 ms regardless of how many IDs are on the bus.

Usage:
    python3 scan_ids.py                         # default port, baud 57600
    python3 scan_ids.py --baudrate 1000000
    python3 scan_ids.py --all-bauds             # try every common baud
    python3 scan_ids.py --port /dev/ttyUSB1
    python3 scan_ids.py --range 1 20            # per-ID ping fallback (slow)

Exit codes:
    0  at least one motor responded
    1  no motors found on any baud tried
    2  port/permission error
"""
import argparse
import sys

from dynamixel_sdk import COMM_SUCCESS, PacketHandler, PortHandler

PROTOCOL = 2.0

# Baud rates Protocol 2.0 X-series can be configured to (value: 0..7 in EEPROM).
COMMON_BAUDS = [57600, 1_000_000, 115200, 2_000_000, 3_000_000, 4_000_000, 4_500_000, 9600]

# Model-number → human name. Dynamixel publishes these in the e-Manual per model.
# Kept minimal and only for models we're confident about; unknown IDs print the
# raw number so the user can look it up without us ever lying.
KNOWN_MODELS = {
    1001: 'XH430-W350',
    1011: 'XH430-W210',
    1021: 'XH430-V210',
    1031: 'XH430-V350',
    1000: 'XH540-W150',
    1010: 'XH540-W270',
    1020: 'XM430-W350',
    1030: 'XM430-W210',
    1060: 'XL430-W250',
    1100: 'XH540-W150',
    1110: 'XH540-W270',
    1120: 'XM540-W150',
    1130: 'XM540-W270',
}


def scan_at_baud(port_path, baud, id_range=None):
    """Try to find motors on ``port_path`` at ``baud``.

    Strategy:
        1. broadcastPing — single-packet; returns {id: [model, fw_version]}.
        2. If ``id_range`` is given, per-ID ping for anything missing.
           Useful on USB adapters that mangle broadcast responses.

    Returns list of tuples: (id, model_number, firmware_version).
    Raises RuntimeError on port open / baud-set failure.
    """
    port = PortHandler(port_path)
    if not port.openPort():
        raise RuntimeError(f"Failed to open port {port_path}")
    try:
        if not port.setBaudRate(baud):
            raise RuntimeError(f"Failed to set baud {baud} on {port_path}")

        packet = PacketHandler(PROTOCOL)

        found = {}

        # broadcast ping
        data, comm = packet.broadcastPing(port)
        if comm != COMM_SUCCESS:
            # Not fatal — some adapters choke on broadcast. Fall through to per-ID.
            sys.stderr.write(
                f"[scan] broadcast at {baud} baud: "
                f"{packet.getTxRxResult(comm)}\n"
            )
        else:
            for mid, info in data.items():
                model = info[0] if len(info) > 0 else None
                fw = info[1] if len(info) > 1 else None
                found[mid] = (model, fw)

        # per-ID fallback
        if id_range is not None:
            lo, hi = id_range
            for mid in range(lo, hi + 1):
                if mid in found:
                    continue
                model, comm, err = packet.ping(port, mid)
                if comm == COMM_SUCCESS and err == 0:
                    found[mid] = (model, None)

        return [(mid, m, fw) for mid, (m, fw) in sorted(found.items())]
    finally:
        port.closePort()


def print_results(baud, motors):
    if not motors:
        print(f"  baud {baud:>7d}  — no response")
        return
    print(f"  baud {baud:>7d}  — {len(motors)} motor(s):")
    for mid, model, fw in motors:
        name = KNOWN_MODELS.get(model, f"unknown (model={model})")
        fw_str = f"  fw=v{fw}" if fw is not None else ""
        print(f"      ID {mid:>3d}   {name}{fw_str}")


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument('--port', default='/dev/ttyUSB0',
                   help="Serial device (default: /dev/ttyUSB0)")
    p.add_argument('--baudrate', type=int, default=57600,
                   help="Baud to scan (default: 57600; ignored if --all-bauds)")
    p.add_argument('--all-bauds', action='store_true',
                   help=f"Try all common bauds: {COMMON_BAUDS}")
    p.add_argument('--range', nargs=2, type=int, metavar=('LO', 'HI'),
                   help="Also per-ID ping IDs in [LO, HI] (slow; for broken broadcast)")
    args = p.parse_args()

    bauds = COMMON_BAUDS if args.all_bauds else [args.baudrate]
    id_range = tuple(args.range) if args.range else None

    print(f"Scanning {args.port}…")
    any_found = False
    try:
        for baud in bauds:
            motors = scan_at_baud(args.port, baud, id_range)
            print_results(baud, motors)
            if motors:
                any_found = True
    except RuntimeError as e:
        print(f"ERROR: {e}", file=sys.stderr)
        return 2

    if not any_found:
        print("\nNo motors found. Check: power, TTL wiring, USB adapter, baudrate.")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
