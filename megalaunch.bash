#!/bin/bash

#
#interactive="shopt -q login_shell && echo 'Login shell' || echo 'Not login shell'
#[[ $- == *i* ]] && echo 'Interactive' || echo 'Not interactive'"
source ~/.bashrc

tmux new -d -s pr2mux 'roscore' \; \
    new-window -n sim 'sleep 2; python3 passthru.py '\; \
    new-window -d -n rviz 'sleep 2; python3 BLEHeartRateLogger/BLEHeartRateLogger.py '\; \
    new-window -d -n fplay 'sleep 5; ffplay /dev/video2; bash'\; \
		attach \;

