#!/bin/bash
####################
# The purpose of this script is to upload all the required folders on the robot's onboard computer.
####################

# ssh in the onboard computer
# search for the McGill_Robosub_2014 folder, if present delete it if not close the ssh connection
# upload the code to the robot


# make sure the script is executed from the right folder:

# Uploads all the required folder on the onboard computer.
downloadBags() {
    echo -e "\r$(tput setaf 3)Downloading all the bags...$(tput sgr 0)"
    scp -r robotics@10.0.0.1:/media/DATA/* ~
    echo "$(tput setaf 3)Download complete.$(tput sgr 0)"
    ssh robotics@10.0.0.1 -t "bash -ic blinky_alert" >& /dev/null
}

downloadBags
