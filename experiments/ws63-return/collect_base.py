"""Read real return reports from the base WS63 or replay an offline fixture."""
import argparse
import csv
from datetime import datetime, timezone
from pathlib import Path
import sys

PREFIX = b"+BCRX:"
FIELDS = ("base_received_utc", "session", "sequence", "device_us", "rssi_dbm")


def parse_report(line):
    line = line.strip()
    if not line.startswith(PREFIX):
        return None
    parts = line[len(PREFIX):].split(b",")
    if len(parts) != 4:
        raise ValueError("wrong number of fields")
    session, sequence, device_us, rssi = map(int, parts)
    if not (1 <= session < 2**32 and 1 <= sequence < 2**32
            and 0 <= device_us < 2**64 and -127 <= rssi <= 20):
        raise ValueError("field out of range")
    return session, sequence, device_us, rssi


def collect(stream, output, *, serial_input=False):
    writer = csv.writer(output)
    writer.writerow(FIELDS)
    highest = {}
    received = 0
    duplicates = 0
    invalid = 0
    pending = bytearray()
    oversized = False
    while True:
        line = stream.readline(256)
        if not line:
            if serial_input:
                continue
            break
        if serial_input:
            if len(pending) + len(line) > 256:
                oversized = True
            if not oversized:
                pending.extend(line)
            if not line.endswith(b"\n"):
                continue
            line = bytes(pending) if not oversized else b""
            pending.clear()
            oversized = False
        try:
            report = parse_report(line)
        except ValueError:
            invalid += 1
            print(f"bad report #{invalid}", file=sys.stderr, flush=True)
            continue
        if report is None:
            continue
        session, sequence, device_us, rssi = report
        if sequence <= highest.get(session, 0):
            duplicates += 1
            continue
        highest[session] = sequence
        now = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        writer.writerow((now, session, sequence, device_us, rssi))
        output.flush()
        received += 1
        print(f"BASE rx={received} dup={duplicates} session={session} seq={sequence} RSSI={rssi} dBm",
              flush=True)
    return received, duplicates, invalid


def main():
    parser = argparse.ArgumentParser()
    source = parser.add_mutually_exclusive_group(required=True)
    source.add_argument("--port")
    source.add_argument("--replay", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    if args.port:
        try:
            import serial
        except ImportError:
            parser.error("serial mode needs pyserial: python -m pip install pyserial")
        stream = serial.Serial(args.port, 115200, timeout=1)
    else:
        stream = args.replay.open("rb")
        print("REPLAY MODE (offline input)", flush=True)
    try:
        with stream, args.output.open("x", encoding="utf-8", newline="") as output:
            collect(stream, output, serial_input=bool(args.port))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
