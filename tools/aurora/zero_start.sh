#!/bin/bash
# Start (or restart) Aurora capture + read-only frame server on the Zero. Not persistent across reboot.
pkill -f '[a]urora_cap --mode' ; pkill -f '[f]rame_server.py'; sleep 1
mkdir -p /tmp/aurora
cd ~/aurora
setsid nohup ./aurora_cap --mode 2 --fps 5 --out /tmp/aurora >/tmp/aurora_cap.log 2>&1 < /dev/null &
setsid nohup python3 frame_server.py --bind 100.96.176.12 --port 8780 --dir /tmp/aurora >/tmp/aurora_frames.log 2>&1 < /dev/null &
sleep 4; pgrep -fa 'aurora_cap|frame_server' | cut -c1-100
