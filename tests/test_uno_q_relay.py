"""Transport and append behavior; no Arduino, Pi, network, or rover is contacted."""
import contextlib
import io
import json
import pathlib
import subprocess
import sys
import tempfile
import unittest
from unittest import mock

ROOT = pathlib.Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "mac"))
import uno_q_relay as relay
import uno_q_log as logger


def record(ms, node="n1"):
    return {"node": node, "ms": ms, "pir": 0, "temp": 23.5}


def line(reading):
    return (json.dumps(reading) + "\n").encode()


class Monitor(io.BytesIO):
    def makefile(self, mode):
        return self

    def send(self, *args):
        raise AssertionError("relay must never send sketch commands")

    sendall = send


class RelayTests(unittest.TestCase):
    def test_partial_start_malformed_wrong_node_and_nonfinite_are_skipped(self):
        payload = (b'"ms":12}\nnot json\n' + line(record(1, "n2"))
                   + b'{"node":"n1","ms":true}\n'
                   + b'{"node":"n1","ms":3,"temp":NaN}\n'
                   + b'{"node":"n1","ms":3,"temp":1e999}\n'
                   + b'\xff\n' + line(record(4)))
        self.assertEqual(list(logger.iter_readings(io.BytesIO(payload), "n1")), [record(4)])

    def test_long_line_is_drained_before_next_record_and_truncated_eof_is_dropped(self):
        payload = b"x" * (logger.MAX_LINE * 3) + b"\n" + line(record(1)) + line(record(2))[:-1]
        self.assertEqual(list(logger.iter_readings(io.BytesIO(payload), "n1")), [record(1)])

    def test_readline_reassembles_fragmented_transport(self):
        class Fragmented(io.RawIOBase):
            def __init__(self, data):
                self.data = io.BytesIO(data)

            def readable(self):
                return True

            def readinto(self, buffer):
                chunk = self.data.read(min(len(buffer), 3))
                buffer[:len(chunk)] = chunk
                return len(chunk)

        with io.BufferedReader(Fragmented(line(record(1)))) as stream:
            self.assertEqual(list(logger.iter_readings(stream, "n1")), [record(1)])

    def test_append_keeps_existing_data(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "n1.jsonl"
            path.write_bytes(line(record(1)))
            with contextlib.redirect_stderr(io.StringIO()):
                count = logger.append_readings(io.BytesIO(line(record(2))), path, "n1")
            self.assertEqual(count, 1)
            self.assertEqual(path.read_bytes(), line(record(1)) + line(record(2)))

    def run_main(self, output, payload, failure=False, count=2):
        original_popen = subprocess.Popen
        children = []

        def local_ssh(argv, **kwargs):
            self.assertEqual(argv[-2], "test@pi")
            if failure:
                command = [sys.executable, "-c", "import sys; sys.stdin.buffer.read(); sys.exit(7)"]
            else:
                # Execute the actual quoted remote command through a local shell.
                command = ["/bin/sh", "-c", argv[-1]]
            child = original_popen(command, stdin=subprocess.PIPE,
                                   stdout=subprocess.PIPE, stderr=subprocess.PIPE)
            children.append(child)
            return child

        args = [str(ROOT / "mac" / "uno_q_relay.py"), "--pi", "test@pi",
                "--output", str(output), "--echo", "--count", str(count)]
        with mock.patch.object(sys, "argv", args), \
                mock.patch.object(relay.socket, "create_connection", return_value=Monitor(payload)) as connect, \
                mock.patch.object(relay.subprocess, "Popen", side_effect=local_ssh), \
                contextlib.redirect_stderr(io.StringIO()):
            status = relay.main()
        connect.assert_called_once_with(("127.0.0.1", 17500), timeout=5)
        results = []
        for child in children:
            with child.stdout, child.stderr:
                results.append((child.stdout.read(), child.stderr.read()))
        return status, results

    def test_complete_relay_writes_only_requested_records_and_echoes_after_logging(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "sensor ' $data.jsonl"
            payload = b"partial}\n" + line(record(1)) + line(record(2)) + line(record(3))
            status, results = self.run_main(path, payload)
            expected = line(record(1)) + line(record(2))
            self.assertEqual(status, 0)
            self.assertEqual(path.read_bytes(), expected)
            self.assertEqual(results[0][0], expected)
            self.assertIn(b"Pi appended 2 reading(s).", results[0][1])

    def test_ssh_or_receiver_failure_is_not_reported_as_success(self):
        with tempfile.TemporaryDirectory() as directory:
            path = pathlib.Path(directory) / "log.jsonl"
            status, _ = self.run_main(path, line(record(1)) + line(record(2)), failure=True)
            self.assertEqual(status, 1)
            self.assertFalse(path.exists())

    def test_empty_or_premature_monitor_eof_fails_bounded_check(self):
        with tempfile.TemporaryDirectory() as directory:
            for payload in (b"", line(record(1))):
                status, _ = self.run_main(pathlib.Path(directory) / "log.jsonl", payload)
                self.assertEqual(status, 1)


if __name__ == "__main__":
    unittest.main()
