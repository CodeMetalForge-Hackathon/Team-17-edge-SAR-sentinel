#!/bin/bash
# (Re)start the navigator on the controller Pi. Usage: pi_start.sh [live|dry]   live = real UGV link (still needs Arm in the UI)
MODE=${1:-live}
pkill -f '[n]avigator/server.py' ; sleep 1
cd ~/forge-overhead/tools/navigator || exit 1
ARGS="--zero http://100.96.176.12:8780 --state-dir ~/forge-nav --port 8790 --rover-mm 170x170 --margin-mm 15 --arrive-tol-mm 50"
[ "$MODE" = live ] && ARGS="$ARGS --enable-motion" || ARGS="$ARGS --dry-run"
setsid nohup python3 /home/khoj/forge-overhead/tools/navigator/server.py $ARGS > ~/forge-overhead/navigator.log 2>&1 < /dev/null &
sleep 3; tail -3 ~/forge-overhead/navigator.log
