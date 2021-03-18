#!/bin/bash
cd
ssh -tt raspi@192.168.1.219 << EOF
roslaunch raspberry_new.launch
EOF