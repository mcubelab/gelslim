#!/bin/bash

tab=" --tab"
options=()

cmds[1]="roscore"
cmds[2]="bash raspi_connect.sh"
#cmds[3]="rosrun rqt_reconfigure rqt_reconfigure"

options1=(--tab -- "bash -c '${cmds[1]} ; bash'")
options2=(--tab -- "bash -c '${cmds[2]} ; bash'")
#options3=(--tab -e "bash -c '${cmds[3]} ; bash'")

gnome-terminal "${options1[@]}"
sleep 5

gnome-terminal "${options2[@]}"
sleep 10

#gnome-terminal "${options3[@]}"
#sleep 5

exit 0
