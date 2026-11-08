# Windows laptop workflow (DRAFT, not tried against the Pi)

The Pi runs the broker and the services. The laptop is a viewer and a place to develop. The teammates' `mac/` scripts are
bash, assume `rsync` and a `khoj@` login, and use `python3`, so on Windows use the equivalents below. **Nothing here has been run
against the real Pi.** `FORGE_MQTT_HOST` is read by `pi/config.py` at `7a9e315` and defaults to `localhost`; that behaviour stays.

| Job | Mac (existing scripts) | Windows (PowerShell, repo root) |
|---|---|---|
| Live map | `bash mac/dashboard.sh [pi-ip]` | `$env:FORGE_MQTT_HOST = "<pi-ip>"; cd pi; python dashboard.py` |
| Reset between judges | R key in the dashboard, or `python pi/reset.py` | R key, or `$env:FORGE_MQTT_HOST = "<pi-ip>"; python pi\reset.py` |
| Copy code to the Pi | `bash mac/sync_to_pi.sh [host]` (rsync) | No rsync by default. Prefer `git pull` on the Pi after a human push, or run the Mac script under WSL or Git Bash if `rsync` exists there. Do not hand-copy over `.venv`, models, baselines or logs |
| Start everything | `~/forge-sar/run_pi.sh [sim|fly|baseline]` on the Pi | The same, on the Pi. The launcher is bash and is not run on Windows |
| Run the checks | not applicable | Core `.venv` from BS-00 APPLY. Contract tests run on the laptop; they do not prove anything about the Pi |

Notes and open items
- The dashboard needs `matplotlib`, `paho-mqtt` and `numpy` in a laptop venv. A human installs them. Agents do not.
- The broker must accept connections from the laptop. Mosquitto 2 listens on localhost only by default unless configured. Check on the Pi (SW-06 documents the result). The Windows firewall may also prompt.
- `mac/dashboard.sh` hard-codes a default Pi address and `mac/sync_to_pi.sh` hard-codes a login. Treat both as one team's values, not facts about every network.
- If the Pi is reached over Tailscale, use that address as `FORGE_MQTT_HOST`.
- A dashboard on the laptop showing events proves the network path, not that sensors, LiDAR, camera or drone work.
