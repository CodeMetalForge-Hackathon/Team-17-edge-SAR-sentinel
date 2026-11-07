#!/bin/bash
# Start the whole pipeline on the Pi. Ctrl+C stops everything.
#
#   ./run_pi.sh          real hardware: brain + every connected Arduino + LiDAR (if found) + drone (dry run)
#   ./run_pi.sh sim      no hardware: brain + simulator + drone (dry run)
#   ./run_pi.sh fly      like the default, but the drone REALLY flies (organizers present only)
#   ./run_pi.sh baseline record the empty-scene LiDAR baseline (do this once, and again if anything moves)
#
# Logs go to logs/<module>.log. Watch one live with:  tail -f logs/brain.log

MODE="${1:-hw}"

# --- edit these for your LiDAR -------------------------------------------
LIDAR_BIN="$HOME/rplidar_sdk/output/Linux/Release/ultra_simple"
LIDAR_PORT="/dev/ttyUSB0"
LIDAR_BAUD=460800            # C1 = 460800, S2 = 1000000
# -------------------------------------------------------------------------

cd "$(dirname "$0")/pi" || exit 1
PY=../.venv/bin/python
mkdir -p ../logs
PIDS=()

start() {  # start <name> <command...>
  local name=$1; shift
  "$@" > "../logs/$name.log" 2>&1 &
  PIDS+=($!)
  echo "  started $name (log: logs/$name.log)"
}

cleanup() {
  echo; echo "stopping..."
  kill "${PIDS[@]}" 2>/dev/null
  pkill -f ultra_simple 2>/dev/null
  wait 2>/dev/null
  echo "all stopped"
  exit 0
}
trap cleanup INT TERM

systemctl is-active --quiet mosquitto || { echo "mosquitto is not running: sudo systemctl start mosquitto"; exit 1; }
[ -f node_iforest.joblib ] || { echo "no anomaly model yet, training a placeholder"; $PY anomaly.py train --synthetic; }

if [ "$MODE" = "baseline" ]; then
  echo "== recording LiDAR baseline: keep the area EMPTY for ~10 s"
  "$LIDAR_BIN" --channel --serial "$LIDAR_PORT" "$LIDAR_BAUD" | $PY -u lidar_node.py --baseline
  pkill -f ultra_simple 2>/dev/null
  exit 0
fi

echo "== starting ($MODE mode)"
start brain $PY -u brain.py

if [ "$MODE" = "sim" ]; then
  start drone $PY -u drone_dispatch.py
  sleep 1
  start sim $PY -u simulate.py
else
  # one bridge per Arduino listed in config.py whose serial port is plugged in
  for node in $($PY -c "import config, os; print(' '.join(n for n, v in config.NODES.items() if os.path.exists(v['port'])))"); do
    start "node_$node" $PY -u node_bridge.py "$node"
  done

  if [ -x "$LIDAR_BIN" ] && [ -e "$LIDAR_PORT" ]; then
    if [ ! -f baseline_scan.npy ]; then
      echo "  no LiDAR baseline yet. Clear the area, then run:  ./run_pi.sh baseline"
    else
      start lidar bash -c "'$LIDAR_BIN' --channel --serial $LIDAR_PORT $LIDAR_BAUD | $PY -u lidar_node.py"
    fi
  else
    echo "  LiDAR skipped (no SDK binary at $LIDAR_BIN or no $LIDAR_PORT)"
  fi

  if [ "$MODE" = "fly" ]; then
    start drone $PY -u drone_dispatch.py --fly --force
  else
    start drone $PY -u drone_dispatch.py
  fi
fi

echo "== running. Dashboard on your laptop: bash mac/dashboard.sh   |   reset between judges: press R there"
echo "== Ctrl+C here to stop everything. Brain output:"
tail -f ../logs/brain.log &
PIDS+=($!)
wait
