#!/bin/bash
####################
# The purpose of this script is to download all the bags from the robot's onboard computer.
####################

downloadBags() {
    echo -e "\r$(tput setaf 3)Downloading all the bags...$(tput sgr 0)"
    scp -r robotics@10.0.0.1:/media/DATA/* ~
    echo "$(tput setaf 3)Download complete.$(tput sgr 0)"
    ssh robotics@10.0.0.1 -t "bash -ic blinky_alert" >& /dev/null
}

downloadBags
