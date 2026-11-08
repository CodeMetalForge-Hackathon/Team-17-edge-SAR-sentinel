# UNO Q on a power bank: direct Wi-Fi logging to Pi

This is the standalone extension of [the USB/Mac relay](UNO_Q_SENSOR_RELAY.md).
The same Python relay runs on UNO Q Linux against port 7500, with a dedicated
SSH key and a separate boot service. The Mac is needed for initial setup only.

```text
UNO Q sensors -> local arduino-router monitor :7500
  -> UNO Q Linux relay -> Wi-Fi hotspot UNOQ-Sensors
  -> Pi 192.168.50.1 SSH -> ~/uno-q-readings/n1.jsonl
```

The UNO Q was connected to the Pi's existing `UNOQ-Sensors` hotspot while keeping
its App Lab USB terminal open. Direct SSH returned `raspberrypi`; a fresh JSON
record at MCU timestamp 533105 ms arrived directly over Wi-Fi. The dedicated key
was subsequently tested with `BatchMode=yes`, and the Pi logger started without
a password and appended zero records for the empty-input login check.

The package checksums and service definition were verified on the UNO Q. Its
packaged foreground relay appended three records successfully. The new service
was then installed, enabled and started by the operator; continuous Pi log
records at MCU timestamps 1450338 through 1451287 ms confirmed wireless logging
from the running service. After giving the existing hotspot profile priority
100 and unlimited activation retries, the operator confirmed a clean reboot
automatically resumed readings without manually joining Wi-Fi. That successful
reboot used Mac USB power. The eight isolated relay tests pass; power-bank-only
startup remains a separate, pending hardware check.

## Prerequisites already established in this setup

- UNO Q Linux user `arduino`, connected to `UNOQ-Sensors`.
- Pi user `khoj`, reachable from the UNO Q at `192.168.50.1:22`.
- Pi logger `/home/khoj/uno-q-relay/uno_q_log.py` installed from the repo.
- Dedicated UNO Q key `/home/arduino/.ssh/uno_q_sensor_ed25519`, kept on that board.
- Pi host key recorded in `/home/arduino/.ssh/known_hosts`.
- The public key appended to the Pi's existing `~/.ssh/authorized_keys` with:

```text
restrict,command="/usr/bin/python3 /home/khoj/uno-q-relay/uno_q_log.py --node n1" PUBLIC_KEY
```

`PUBLIC_KEY` above is a placeholder, not an installation command. Existing keys
are retained. This key can run only the installed logger; its forced command
fixes the node and output path. Do not pass `--echo` or `--output` expecting to
override that forced command. No private keys or passwords belong in this repo
or a deployment archive. [OpenSSH authorized-key options](https://man.openbsd.org/sshd.8#AUTHORIZED_KEYS_FILE_FORMAT)

The initial unauthenticated raw GitHub download returned 404; its cause was not
established. The logger was instead copied from the Mac over existing Pi SSH,
with shell noclobber protection, and its `--help` output verified. Private repo
access is not required on either board once the files are copied.

## Deploy code to the UNO Q while USB is still connected

From the **Mac's repository root**, create a deployment archive using only the
three program/configuration files and a checksum manifest:

```bash
python3 - <<'PY'
from pathlib import Path
import hashlib
import io
import tarfile

names = ("mac/uno_q_relay.py", "pi/uno_q_log.py",
         "systemd/uno-q-sensor-relay.service")
files = {name: Path(name).read_bytes() for name in names}
files["SHA256SUMS"] = "".join(
    hashlib.sha256(data).hexdigest() + "  " + name + "\n"
    for name, data in files.items()
).encode()
with tarfile.open("uno-q-standalone.tar.gz", "x:gz") as archive:
    for name, data in files.items():
        entry = tarfile.TarInfo(name)
        entry.mode = 0o644
        entry.size = len(data)
        archive.addfile(entry, io.BytesIO(data))
print("Created uno-q-standalone.tar.gz")
PY
```

It refuses to overwrite an existing archive. Its contents are:

```text
mac/uno_q_relay.py
pi/uno_q_log.py
systemd/uno-q-sensor-relay.service
```

Use the exact ADB executable already running for App Lab, as described in the
USB relay guide; a different client can replace the existing ADB server. From
the Mac, copy the prepared archive with that client and device ID:

```bash
UNO_Q_ADB="$HOME/Library/Arduino15/packages/arduino/tools/adb/32.0.0/adb"
"$UNO_Q_ADB" -P 5037 -s 3615409405 push uno-q-standalone.tar.gz /home/arduino/uno-q-standalone.tar.gz
```

In the **UNO Q App Lab terminal**, unpack without overwriting existing files:

```bash
mkdir -p "$HOME/uno-q-relay"
tar --keep-old-files -xzf "$HOME/uno-q-standalone.tar.gz" -C "$HOME/uno-q-relay"
cd "$HOME/uno-q-relay"
sha256sum -c SHA256SUMS
```

Stop on extraction errors or checksum failures. This installs only user files;
the unit file in the home directory is not active yet. Its fixed user, paths,
Pi address and node must match your setup.

## Foreground check before enabling boot startup

From the **UNO Q terminal**:

```bash
python3 "$HOME/uno-q-relay/mac/uno_q_relay.py" --pi khoj@192.168.50.1 --node n1 --monitor-port 7500 --identity "$HOME/.ssh/uno_q_sensor_ed25519" --count 3
```

The specific key is selected with `IdentitiesOnly=yes`, password prompts are
disabled with `BatchMode=yes`, and `StrictHostKeyChecking=yes` requires the
existing trusted host entry. A failure exits instead of accepting a new host or
using a password. Confirm the Pi log's last three records from the **Pi terminal**:

```bash
tail -n 3 ~/uno-q-readings/n1.jsonl
```

## Install the separate boot service on UNO Q

First validate it on the actual Linux system:

```bash
systemd-analyze verify "$HOME/uno-q-relay/systemd/uno-q-sensor-relay.service"
```

If clean, install the new unit while refusing to overwrite an existing unit:

```bash
sudo test ! -e /etc/systemd/system/uno-q-sensor-relay.service &&
sudo install -m 644 "$HOME/uno-q-relay/systemd/uno-q-sensor-relay.service" /etc/systemd/system/uno-q-sensor-relay.service &&
sudo systemctl daemon-reload &&
sudo systemctl enable --now uno-q-sensor-relay.service
```

`daemon-reload` reads unit definitions; this sequence starts only the new relay.
It does not restart `arduino-router` or alter any Pi service. The unit runs as
`arduino`, waits for the normal network/router boot ordering, and retries its
own process five seconds after an exit. It does not start/stop the router or
manage Wi-Fi profiles. [systemd service reference](https://www.freedesktop.org/software/systemd/man/latest/systemd.service.html)

Verify its status and new Pi log records before power cycling:

```bash
systemctl status uno-q-sensor-relay.service --no-pager --full
```

Configure **only the UNO Q's saved hotspot profile**, using the UUID reported
when it was connected. Give it a stronger preference than the default-priority
networks and retry failed activations indefinitely:

```bash
sudo nmcli connection modify 178185bb-7a12-49fd-8998-8d737d701997 \
  connection.autoconnect yes \
  connection.autoconnect-priority 100 \
  connection.autoconnect-retries 0

nmcli -f connection.autoconnect,connection.autoconnect-priority,connection.autoconnect-retries \
  connection show 178185bb-7a12-49fd-8998-8d737d701997
```

Expected values are `yes`, `100`, and `0 (forever)`. This does not activate or
disconnect a profile immediately, change the Pi's AP, or delete other saved
networks. In this setup, `connection.permissions` was unrestricted and PSK flags
were `0 (none)`, rather than a password owned by a desktop login agent.

Higher priority matters when multiple profiles are available for autoconnection;
it does not preempt an already active connection. Keep the Pi hotspot running
before rebooting the UNO Q. If other saved profiles have priority 100 or higher,
inspect their priorities before choosing a higher value for this hotspot.
[NetworkManager connection settings](https://networkmanager.dev/docs/api/latest/settings-connection.html)

With the Pi log viewer running, use `sudo reboot` in the UNO Q terminal and
confirm new readings resume without manually joining Wi-Fi. This clean-reboot
test passed under Mac USB power after the profile changes. A relay retry alone
cannot repair association to the wrong network; this setup changes the intended
hotspot profile rather than modifying or removing the others.

## Move to a power bank and verify boot

Use a USB-C power bank and cable supporting **5 V at 3 A**.
[Arduino power specifications](https://docs.arduino.cc/tutorials/uno-q/power-specification?queryID=2aa1d01e13483af851bf99090e9e0e3c)
In the UNO Q terminal, halt **the UNO Q only** before transferring its power:

```bash
sudo halt
```

Wait approximately 10–15 seconds before disconnecting USB. App Lab losing its
connection is expected; LEDs may stay lit. Arduino recommends `halt` because
other shutdown commands can cause this board to restart automatically.
[Arduino shutdown guidance](https://docs.arduino.cc/tutorials/uno-q/debian-guide/#shutting-down-your-uno-q-safely)
Keep the Pi powered on. Move the UNO Q cable to the power bank, allow Linux to
boot, then inspect fresh records from the Pi terminal:

```bash
tail -n 0 -f ~/uno-q-readings/n1.jsonl
```

Do not manually join Wi-Fi during this test. Fresh records after that boot
establish operation without the Mac/USB relay.
MCU `ms` may reset on reboot; do not mistake the last old record for new data.
This still records sensor data only, without MQTT or rover policy integration.

## Stop and limits

On UNO Q, stop and disable only this new service with:

```bash
sudo systemctl disable --now uno-q-sensor-relay.service
```

The service retries monitor/SSH failures; it cannot log during an outage and
does not replay missed records. An in-flight record may be lost on shutdown.
The Pi append log has no rotation or disk quota. Only run one forwarder per
sensor node; stop the older Mac foreground relay before enabling this one.
The router, Wi-Fi/rover configuration and other SSH authorized keys are retained.
