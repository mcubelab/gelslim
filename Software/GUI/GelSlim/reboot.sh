#!/bin/bash
tab=" --tab"
options=()
cmds[1]="cd
ssh -tt raspi@192.168.1.219 << EOF
echo raspi | sudo -S reboot
EOF"
options1=(--tab -e "bash -c '${cmds[1]} ; bash'")
gnome-terminal "${options1[@]}"
sleep 5