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
	echo "Starting uploading all the required folders..."

	echo "Uploading the 'catkin/src' folder..."
	sshpass -p 'elgordo21' scp -r catkin_ws/src robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
	echo "Uploading the 'catkin/launch' folder..."
	sshpass -p 'elgordo21' scp -r catkin_ws/launch robotics@10.0.0.1:~/McGill_RoboSub_2014/catkin_ws
	echo "Uploading the 'Arduino' folder..."
	sshpass -p 'elgordo21' scp -r Arduino robotics@10.0.0.1:~/McGill_RoboSub_2014/
	echo "Uploading the 'scripts' folder..."
	sshpass -p 'elgordo21' scp -r scripts robotics@10.0.0.1:~/McGill_RoboSub_2014/
	echo "Uploading the 'setup_scripts' folder..."
	sshpass -p 'elgordo21' scp -r setup_scripts robotics@10.0.0.1:~/McGill_RoboSub_2014/
}

uploadRepository

echo "The execution of this script is now completed."
exit
