#!/bin/bash
# Live map on the laptop, reading from the broker on the Pi.
#   bash mac/dashboard.sh                 # default Pi IP below
#   bash mac/dashboard.sh 100.119.240.31  # e.g. the Tailscale IP if you're on another network
# Needs: pip install matplotlib paho-mqtt numpy
export FORGE_MQTT_HOST="${1:-10.1.200.210}"
cd "$(dirname "$0")/../pi" && python3 dashboard.py
