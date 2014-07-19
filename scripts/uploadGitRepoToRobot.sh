#!/bin/bash
####################
# The purpose of this script is to upload all the required folders on the robot's onboard computer.
####################

# ssh in the onboard computer
# search for the McGill_Robosub_2014 folder, if present delete it if not close the ssh connection
# upload the code to the robot


# make sure the script is executed from the right folder:

# Uploads all the required folder on the onboard computer.
uploadRepository() {
    echo "$(tput setaf 1)Removing old files from Asimov...$(tput sgr 0)"
    ssh robotics@10.0.0.1 -t "cd ~/McGill_RoboSub_2014/ && rm -rf catkin_ws/src catkin_src/launch Arduino scripts setup" >& /dev/null

    echo -e "\r$(tput setaf 4)Uploading all the required files...$(tput sgr 0)"

    echo "$(tput setaf 3)Uploading the 'catkin/src' folder...$(tput sgr 0)"
    scp -r $ROBOTIC_PATH/catkin_ws/src robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
    echo "$(tput setaf 7)Uploading the 'catkin/launch' folder...$(tput sgr 0)"
    scp -r $ROBOTIC_PATH/catkin_ws/launch robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
    echo "$(tput setaf 4)Uploading the 'Arduino' folder...$(tput sgr 0)"
    scp -r $ROBOTIC_PATH/Arduino robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "$(tput setaf 3)Uploading the 'scripts' folder...$(tput sgr 0)"
    scp -r $ROBOTIC_PATH/scripts robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "$(tput setaf 7)Uploading the 'setup' folder...$(tput sgr 0)"
    scp -r $ROBOTIC_PATH/setup robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "$(tput setaf 4)Upload complete.$(tput sgr 0)"
    ssh robotics@10.0.0.1 -t "bash -ic blinky_alert" >& /dev/null
}

# Runs catkin_make remotely
compile() {
    echo "$(tput setaf 1)Compiling...$(tput sgr 0)"
    ssh robotics@10.0.0.1 -t "bash -ic 'cd ~/McGill_RoboSub_2014/catkin_ws/ && catkin_alert'"
    echo "$(tput setaf 1)Compilation complete.$(tput sgr 0)"
}

uploadRepository

if [ $# -eq 1 ] && [ "$1" = "--no-compile" ]; then
    echo "$(tput setaf 1)Skipping compilation.$(tput sgr 0)"
else
    compile
fi

echo "$(tput setaf 4)Done.$(tput sgr 0)"
