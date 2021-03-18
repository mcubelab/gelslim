#!/bin/bash

tab=" --tab"
options=()

cmds[1]="rosrun rqt_reconfigure rqt_reconfigure"

options1=(--tab -e "bash -c '${cmds[1]} ; bash'")

gnome-terminal "${options1[@]}"

exit 0
