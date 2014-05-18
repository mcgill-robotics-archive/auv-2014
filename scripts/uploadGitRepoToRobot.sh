#!/bin/bash

sshpass -p 'elgordo21' scp -r ../catkin_ws/src robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
sshpass -p 'elgordo21' scp -r ../Arduino robotics@10.0.0.1:~/McGill_RoboSub_2014/
sshpass -p 'elgordo21' scp -r ../setup robotics@10.0.0.1:~/McGill_RoboSub_2014/
