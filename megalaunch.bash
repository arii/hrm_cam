#!/bin/bash

#
interactive="shopt -q login_shell && echo 'Login shell' || echo 'Not login shell'
[[ $- == *i* ]] && echo 'Interactive' || echo 'Not interactive'"
source ~/.bashrc

tmux new -d -s pr2mux 'echo "roscore"; roscore; bash' \; \
    new-window -d -n sim 'eco "cv2"; sleep 1; python passthru.py '\; \
    new-window -d -n rviz 'echo "hrmpub"; sleep 1; python BLEHeartRateLogger/BLEHeartRateLogger.py '\; \
    new-window -d -n fplay 'echo "ffplay"; sleep 5; ffplay /dev/video1 ; bash'\; \
		attach \;

