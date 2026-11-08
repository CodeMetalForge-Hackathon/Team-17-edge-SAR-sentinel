#!/usr/bin/env python3
"""Append validated UNO Q JSON lines from stdin. No MQTT or actuator integration."""
import argparse
import json
import pathlib
import re
import sys

MAX_LINE = 8192


def node_id(value):
    if not re.fullmatch(r"[A-Za-z0-9_-]{1,64}", value):
        raise argparse.ArgumentTypeError("node must contain only letters, digits, _ or -")
    return value


def reject_constant(value):
    raise ValueError("non-finite JSON number: " + value)


def iter_readings(stream, node):
    """Handle fragmented transport, a partial first line, and oversized/bad records."""
    while True:
        line = stream.readline(MAX_LINE + 1)
        if not line:
            return
        if len(line) > MAX_LINE:
            while line and not line.endswith(b"\n"):
                line = stream.readline(MAX_LINE + 1)
            continue
        # EOF in the middle of a record is not a complete JSON line.
        if not line.endswith(b"\n"):
            return
        try:
            reading = json.loads(line, parse_constant=reject_constant)
            if not isinstance(reading, dict) or reading.get("node") != node:
                continue
            ms = reading.get("ms")
            if type(ms) is not int or ms < 0:
                continue
            # Also rejects finite-looking literals such as 1e999 that decode as infinity.
            json.dumps(reading, allow_nan=False)
        except (ValueError, UnicodeDecodeError, RecursionError):
            continue
        yield reading


def append_readings(stream, output, node, echo=False):
    output = pathlib.Path(output).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    count = 0
    with output.open("a", encoding="utf-8") as log:
        print("Pi sensor log: " + str(output), file=sys.stderr, flush=True)
        for reading in iter_readings(stream, node):
            line = json.dumps(reading, allow_nan=False) + "\n"
            log.write(line)
            log.flush()
            count += 1
            if echo:
                print(line, end="", flush=True)
    print("Pi appended %d reading(s)." % count, file=sys.stderr, flush=True)
    return count


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--node", type=node_id, default="n1")
    parser.add_argument("--output", help="Pi path; default: ~/uno-q-readings/<node>.jsonl")
    parser.add_argument("--echo", action="store_true", help="also print stored readings")
    args = parser.parse_args()
    output = args.output or pathlib.Path.home() / "uno-q-readings" / (args.node + ".jsonl")
    try:
        append_readings(sys.stdin.buffer, output, args.node, args.echo)
    except KeyboardInterrupt:
        return 130
    except OSError as exc:
        print("Pi logger failed: " + str(exc), file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
