#!/bin/bash
echo "[INFO] Environment executed..."
. ~/suction_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.83:11311/
export ROS_IP=192.168.1.219
export ROS_HOSTNAME=192.168.1.219
export ROSLAUNCH_SSH_UNKNOWN=1

PROMPT_COMMAND='history -a'
history -a

# sorting in old style
LC_COLLATE="C"
export LC_COLLATE
ulimit -c unlimited
export HISTTIMEFORMAT="%d/%m/%y %T "

