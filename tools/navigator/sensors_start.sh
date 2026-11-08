#!/bin/bash
# (Re)start the UNO Q -> chance-of-life pipeline on the controller Pi (run next to pi_start.sh).
#   UNO Q relay (already a service on the UNO Q) -> ~/uno-q-readings/n1.jsonl
#   pi/model_feed.py   tags rows with stop.txt (S1, E2... written by the navigator during each dwell) -> live.csv
#   model/survivor.py  scores each stop -> forge/event (SURVIVOR_PROB) + results.json (shown on the navigator page)
# Usage: sensors_start.sh [calib.csv]     default calib: model/calib.csv (SIMULATED baselines; record real empty stops on site)
ROOT=$(cd "$(dirname "$0")/../.." && pwd)
DIR=~/uno-q-readings
CALIB=${1:-$ROOT/model/calib.csv}
mkdir -p "$DIR"
pkill -f '[m]odel_feed.py' ; pkill -f '[s]urvivor.py' ; sleep 1
echo "-" > "$DIR/stop.txt"
[ -s "$DIR/live.csv" ] && mv "$DIR/live.csv" "$DIR/live.$(date +%H%M%S).csv"     # fresh run: no replay of old stops
[ -f "$DIR/results.json" ] && mv "$DIR/results.json" "$DIR/results.$(date +%H%M%S).json"
setsid nohup python3 -u "$ROOT/pi/model_feed.py" --out "$DIR/live.csv" > "$DIR/model_feed.log" 2>&1 < /dev/null &
for i in 1 2 3 4 5 6 7 8 9 10; do [ -s "$DIR/live.csv" ] && break; sleep 0.5; done
setsid nohup python3 -u "$ROOT/model/survivor.py" "$DIR/live.csv" --live --calib "$CALIB" --mqtt localhost \
    > "$DIR/survivor.log" 2>&1 < /dev/null &
sleep 2
pgrep -fa 'model_feed.py|survivor.py' | cut -c1-120
echo "readings: $(wc -l < "$DIR/n1.jsonl" 2>/dev/null || echo 0) lines in $DIR/n1.jsonl (should grow ~10/s while the UNO Q is on)"
tail -2 "$DIR/survivor.log"
