#!/usr/bin/env python3
"""Stream local UNO Q monitor JSON to Pi SSH (Mac USB forward or UNO Q Linux)."""
import argparse
import pathlib
import shlex
import socket
import subprocess
import sys

RECEIVER = pathlib.Path(__file__).resolve().parents[1] / "pi" / "uno_q_log.py"
sys.path.insert(0, str(RECEIVER.parent))
from uno_q_log import iter_readings, node_id


def positive_int(value):
    number = int(value)
    if number <= 0:
        raise argparse.ArgumentTypeError("must be positive")
    return number


def ssh_command(args):
    # Only Python code and quoted arguments go into the remote shell command.
    # No script is installed on the Pi; its stdin carries the sensor records.
    command = ["python3", "-u", "-c", RECEIVER.read_text(encoding="utf-8"),
               "--node", args.node]
    if args.output:
        command += ["--output", args.output]
    if args.echo:
        command.append("--echo")
    ssh = ["ssh", "-T", "-o", "ConnectTimeout=5",
           "-o", "ServerAliveInterval=10", "-o", "ServerAliveCountMax=2"]
    if args.identity:
        ssh += ["-i", str(pathlib.Path(args.identity).expanduser()),
                "-o", "IdentitiesOnly=yes", "-o", "BatchMode=yes",
                "-o", "StrictHostKeyChecking=yes"]
    return ssh + ["--", args.pi, shlex.join(command)]


def relay(stream, destination, node, count=None):
    import json
    sent = 0
    for reading in iter_readings(stream, node):
        destination.write((json.dumps(reading, allow_nan=False) + "\n").encode("utf-8"))
        destination.flush()
        sent += 1
        if count is not None and sent >= count:
            break
    return sent


def finish(child, abort=False):
    try:
        child.stdin.close()
    except (BrokenPipeError, OSError):
        pass
    if abort and child.poll() is None:
        child.terminate()
    try:
        return child.wait(timeout=30 if not abort else 5)
    except subprocess.TimeoutExpired:
        child.kill()
        child.wait()
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--pi", required=True, help="existing SSH login, e.g. khoj@100.119.240.31")
    parser.add_argument("--node", type=node_id, default="n1")
    parser.add_argument("--monitor-port", type=positive_int, default=17500)
    parser.add_argument("--output", help="output path on Pi, default: ~/uno-q-readings/<node>.jsonl")
    parser.add_argument("--count", type=positive_int, help="stop after this many valid records")
    parser.add_argument("--echo", action="store_true", help="show readings after Pi writes them")
    parser.add_argument("--identity", help="SSH key for unattended login; requires a known host key")
    args = parser.parse_args()
    if args.pi.startswith("-") or any(c.isspace() for c in args.pi):
        parser.error("--pi must be a single SSH destination")
    if args.monitor_port > 65535:
        parser.error("--monitor-port must be between 1 and 65535")
    if args.identity and not pathlib.Path(args.identity).expanduser().is_file():
        parser.error("--identity must name an existing SSH private key")

    child = None
    result = 0
    try:
        # The monitor connection is receive-only; no ADB invocation or sketch commands.
        with socket.create_connection(("127.0.0.1", args.monitor_port), timeout=5) as monitor:
            child = subprocess.Popen(ssh_command(args), stdin=subprocess.PIPE)
            print("Relaying %s to %s; Ctrl+C stops this relay." % (args.node, args.pi),
                  file=sys.stderr, flush=True)
            with monitor.makefile("rb") as stream:
                sent = relay(stream, child.stdin, args.node, args.count)
            if sent == 0:
                print("Monitor closed without a valid sensor reading.", file=sys.stderr)
                result = 1
            elif args.count is not None and sent < args.count:
                print("Monitor closed before the requested record count.", file=sys.stderr)
                result = 1
            else:
                print("Forwarded %d reading(s)." % sent, file=sys.stderr)
    except KeyboardInterrupt:
        result = 130
    except (OSError, ValueError, subprocess.SubprocessError) as exc:
        print("Relay stopped: " + str(exc), file=sys.stderr)
        result = 1
    finally:
        if child is not None:
            status = finish(child, abort=result != 0)
            if result == 0 and status != 0:
                result = 1
                print("Pi SSH/logger exited with status %s." % status, file=sys.stderr)
    return result


if __name__ == "__main__":
    sys.exit(main())
