#!/bin/bash
# Copy your latest code from this laptop to the Pi. Run from anywhere on the Mac:
#   bash mac/sync_to_pi.sh                # uses host "raspberrypi"
#   bash mac/sync_to_pi.sh 10.1.200.210   # or the Pi's IP
# Doesn't touch the Pi's .venv, trained model, LiDAR baseline, logs, or logged data.
PI_HOST="${1:-raspberrypi}"
REPO="$(cd "$(dirname "$0")/.." && pwd)"
rsync -av --exclude .git --exclude .venv --exclude logs --exclude cache --exclude __pycache__ \
  --exclude '*.joblib' --exclude '*.npy' --exclude 'normal_features.csv' \
  "$REPO/" "khoj@${PI_HOST}:~/forge-sar/"
ssh "khoj@${PI_HOST}" "chmod +x ~/forge-sar/run_pi.sh"
echo "synced. On the Pi: ~/forge-sar/run_pi.sh"
