#!/bin/bash

echo "This script will launch all the sensor nodes.";

echo "It will now ssh to the onboard computer. Will timout after 15 seconds if the connection is not established...";
#sshpass -p 'elgordo21' ssh -o ConnectTimeout=15 robotics@10.0.0.1
#result=$(sshpass -p 'elgordo21' ssh -o ConnectTimeout=15 robotics@10.0.0.1 2>&1)
#echo $result;
echo "You should be on the onboard computer as of now.";
read -t 10 -p "Press enter to launch the tmux session or wait 10 seconds."

tmux new-session -d "teamocil sample" \; attach