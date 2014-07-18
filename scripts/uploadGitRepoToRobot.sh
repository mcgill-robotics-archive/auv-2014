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
    echo "Removing old files from Asimov..."
    sshpass -p 'elgordo21' ssh robotics@10.0.0.1 -t "cd ~/McGill_RoboSub_2014/ && rm -rf catkin_ws/src catkin_src/launch Arduino scripts setup" >& /dev/null

    echo -e "\rUploading all the required files..."

    echo "Uploading the 'catkin/src' folder..."
    sshpass -p 'elgordo21' scp -r ../catkin_ws/src robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
    echo "Uploading the 'catkin/launch' folder..."
    sshpass -p 'elgordo21' scp -r ../catkin_ws/launch robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
    echo "Uploading the 'Arduino' folder..."
    sshpass -p 'elgordo21' scp -r ../Arduino robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "Uploading the 'scripts' folder..."
    sshpass -p 'elgordo21' scp -r ../scripts robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "Uploading the 'setup' folder..."
    sshpass -p 'elgordo21' scp -r ../setup robotics@10.0.0.1:~/McGill_RoboSub_2014
    echo "Upload complete."
    sshpass -p 'elgordo21' ssh robotics@10.0.0.1 -t "bash -ic blinky_alert" >& /dev/null

}

# Runs catkin_make remotely
compile() {
    echo "Compiling..."
    sshpass -p 'elgordo21' ssh robotics@10.0.0.1 -t "bash -ic 'cd ~/McGill_RoboSub_2014/catkin_ws/ && catkin_alert'"
    echo "Compilation complete."
}

uploadRepository

if [ $# -eq 1 ] && [ "$1" = "--no-compile" ]; then
    echo "Skipping compilation."
else
    compile
fi

echo "Done."
