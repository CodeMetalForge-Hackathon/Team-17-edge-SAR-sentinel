# UNO Q sensor readings: USB through Mac to Raspberry Pi

This guide records the connection verified on 2026-11-08. The existing sketch's
JSON reaches UNO Q Linux through the router's monitor, then reaches a separate
Pi log through the Mac's USB connection and the Pi's existing Tailscale/SSH link.
The Mac is part of this path and must remain awake and connected by USB.

```text
sensor_node.ino (NODE_SERIAL / Serial)
  -> UNO Q arduino-router monitor, 127.0.0.1:7500
  -> existing App Lab ADB connection over USB
  -> Mac 127.0.0.1:17500
  -> Mac SSH over existing Tailscale connection
  -> Pi ~/uno-q-readings/n1.jsonl
```

The new scripts use Python 3.9+ standard libraries and the existing SSH client.
They collect sensor data only. They do not publish MQTT features, send sensor
commands, or connect the readings to rover/drone policy. No service installation,
firmware upload, or Wi-Fi, routing, firewall or Tailscale change is part of setup.

## Confirmed configuration and limits

| Item | Observed value |
| --- | --- |
| UNO Q node | `n1` |
| UNO Q USB device ID | `3615409405` |
| UNO Q router monitor | `127.0.0.1:7500` |
| Mac USB forward | `127.0.0.1:17500` |
| Pi SSH destination from Mac | `khoj@100.119.240.31` |
| Pi sensor log | `~/uno-q-readings/n1.jsonl` |

These addresses and device ID describe the tested setup; substitute yours if
different. UNO Q `10.1.200.27` could not connect to Pi `10.1.200.210` on ports
8765 or 22, or to Pi AP address `192.168.50.1:8765`. The cause of the reachability
failure was not established. Mac-to-Pi SSH over Tailscale succeeded.

Pi port 8765 is the overhead viewer, port 8088 is HQ, and port 8790 is the
navigator. The inspected viewer and HQ scripts have no `/sensors` route. Leave
them running unchanged. The navigator was started with motion enabled; it is
not part of this relay. A previously assumed HTTP sensor receiver at
`192.168.50.1:8765/sensors` was incorrect.

On the installed UNO Q CLI, both `arduino-app-cli app monitor` and `app ps`
panic with `not implemented`. Default system Python also lacks `arduino`.
Neither command nor that module is needed here. Do not open `/dev/ttyHS1`
directly: it belongs to `arduino-router`.

UNO Q core versions differ: from Zephyr core 0.55.0, `Serial` aliases the
Bridge-backed `Monitor`; older cores used the UART pins. The running sketch's
monitor output was verified directly, without rebuilding to infer its core
version. `Serial1` on current cores targets hardware pins, not this Linux path.
See the [Arduino core migration guide](https://support.arduino.cc/hc/en-us/articles/27251870677916-Migrating-to-Zephyr-core-0-55-0-on-UNO-Q)
and [UNO Q manual](https://docs.arduino.cc/tutorials/uno-q/user-manual/).

## 1. Reuse App Lab's existing ADB server on the Mac

Keep App Lab open. Run these in the **Mac terminal**, not a Pi or UNO Q shell:

```bash
ps -axo pid,command | grep '[a]db'
```

Identify the ADB server PID and its executable before invoking any ADB client:

```bash
lsof -a -p SERVER_PID -d txt -Fn | grep '/adb$'
```

Replace `SERVER_PID` with the actual PID, not the shell PID. The output begins
with `n`; the remainder is the executable path. In the tested setup, the running
server and App Lab shell both used:

```text
$HOME/Library/Arduino15/packages/arduino/tools/adb/32.0.0/adb
```

Use the exact running server's executable. A mismatched ADB client can kill and
replace an existing server even for `devices -l`; this happened to a separate
Pi ADB server during investigation. Do not run `kill-server`, `start-server`,
`usb`, `tcpip`, or a different generic `adb` to set up this relay.

With the verified executable and the correct device ID:

```bash
UNO_Q_ADB="$HOME/Library/Arduino15/packages/arduino/tools/adb/32.0.0/adb"
"$UNO_Q_ADB" -P 5037 -s 3615409405 forward --no-rebind tcp:17500 tcp:7500
```

This printed `17500` in the tested setup. `--no-rebind` refuses to replace an
existing forward. If it reports an existing rule, inspect with the same matching
client's `forward --list`; retain it only if it already maps this device's
`tcp:17500` to `tcp:7500`. Other port-use errors require an unused local port,
also supplied to the relay's `--monitor-port`. USB disconnection or an App Lab /
ADB restart may require recreating the mapping. [ADB command reference](https://android.googlesource.com/platform/packages/modules/adb/+/refs/heads/main/docs/user/adb.1.md)

## 2. Check the Mac-to-Pi link

From the Mac:

```bash
nc -vz -G 5 100.119.240.31 22
```

The tested result was `Connection ... succeeded!`. SSH authentication is still
required when running the relay; use the Pi's existing login and normal host-key
verification. The relay does not store keys, passwords or Tailscale tokens.

## 3. Run the relay from the Mac's Team-17 repository root

First capture three records and exit:

```bash
python3 mac/uno_q_relay.py --pi khoj@100.119.240.31 --node n1 --count 3 --echo
```

Then run continuously in the foreground:

```bash
python3 mac/uno_q_relay.py --pi khoj@100.119.240.31 --node n1 --echo
```

Enter your Pi SSH password if prompted. The Mac runs the checked-in
`pi/uno_q_log.py` source in a temporary remote Python process over SSH; no script
is installed on the Pi. It appends validated records to the Pi log, flushes each
line, and optionally echoes the stored record to your terminal. Existing log
contents are retained. `--output` selects another path **on the Pi**.

The reader skips a partial first line, malformed/oversized records, records for
another node, non-finite numeric values and incomplete final lines. It preserves
the sensor fields and MCU `ms` timestamp. That timestamp is milliseconds since
MCU boot, not wall-clock time. The sketch requests a 100 ms period; observed
records were roughly 180–206 ms apart, so do not assume exactly 10 Hz.

## 4. Verify the file on the Pi

In the **Pi terminal**:

```bash
tail -n 3 ~/uno-q-readings/n1.jsonl
```

During the hardware check, `Received on: raspberrypi` confirmed a single JSON
record arrived over this path. A subsequent foreground stream was stopped with
Ctrl+C, and the Pi file contained the same final records at MCU timestamps
2595561, 2595767 and 2595946 ms. The reusable scripts additionally have automated
transport, validation, append and failure checks; their packaged implementation
has not been rerun on the physical boards as part of the GitHub upload.

## Stop, resume and limitations

Ctrl+C stops the foreground relay. The logger touches only the selected sensor
log, and its shutdown does not call a rover STOP handler. Existing router,
viewer, HQ and navigator processes remain separate. Resume with the same relay
command while the USB forward and Mac-to-Pi SSH connection are available.

To remove only this USB forward, use the matching ADB executable:

```bash
"$UNO_Q_ADB" -P 5037 -s 3615409405 forward --remove tcp:17500
```

The scripts do not restart services or reconnect automatically after loss. A
five-second monitor idle timeout, SSH error, or output write error ends the
relay. There is no durable resend queue, exactly-once delivery, file rotation or
bounded disk quota. Repeated sessions append, so choose storage/rotation before
leaving it running unattended. Data accumulated while waiting for SSH login may
arrive late. Neither MQTT `pi/node_bridge.py` nor the rover apps ingest this log;
that integration remains separate.

## Uploaded sensor sketch

`arduino/sensor_node/sensor_node.ino` contains the uploaded `sensor_node(2).ino`
with a clarified port comment; its runtime code is unchanged from the upload.
Grove shield voltage is 3V3, vibration is active LOW and binary per output
window, water uses `INPUT_PULLUP`, and DHT11 is enabled on D8 (`DHT.h` required).
`NODE_SERIAL` defaults to `Serial` and can be overridden at build time.

There is no physical flame sensor in this build. `flame` is a heat-rise flag,
set at 5 C above the baseline taken after startup; it is not a reliable fire
detector. `B` triggers the beacon, `A` blinks acknowledgment, and `Z` resets the
heat baseline. The relay sends none of these characters. No firmware upload was
needed to establish the data path, and none is performed by these tools.

Run the isolated checks from the repo root:

```bash
python3 -m unittest discover -s tests -p 'test_uno_q_relay.py' -v
```
