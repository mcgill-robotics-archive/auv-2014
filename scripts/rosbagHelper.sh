#!/bin/bash
############################
# This script allows a used to ROSBAG specific, or all, topics of certain components very easily.
############################

# Variable initialization
command_arguments=''

# Displays the usage of this script.
usage() {
    echo USAGE: rosbagHelper.sh OPTION
    echo "options: -a : all topics"  
    echo "         -i : imu pose data"
    echo "         -t : all temperature"
    echo "         -c : camera data; i.e. camera info, image rectified and color image from up and down cameras"
    echo "         -d : depth sensor data"
    echo "         -h : prints this menu"
}

# Starts ROSBag with the given parameters.
startRosBagWithParameters() {
	rosbag record $command_arguments -O ~/bag-$(date +%s)
}

# Handls the input of the scripts.
handleInputParameters() {
	for arg in "$@"; do
		case ${arg} in
			'-a' )
				command_arguments="${command_arguments} status/usb electrical_interface/batteryVoltage1 electrical_interface/batteryVoltage2 electrical_interface/pressure electrical_interface/motor electrical_interface/solenoid state_estimation/raw"
				command_arguments="${command_arguments} state_estimation/pose"
				command_arguments="${command_arguments} electrical_interface/depth"
				command_arguments="${command_arguments} status/temperature electrical_interface/temperature electrical_interface/temperature1 electrical_interface/temperature2 electrical_interface/temperature3 electrical_interface/temperature4 electrical_interface/temperature5"
				command_arguments="${command_arguments} status/temperature electrical_interface/temperature"
				command_arguments="${command_arguments} camera_down/camera_out/camera_info camera_down/image_rect camera_front_left/camera_out/camera_info camera_front_left/image_rect"
				startRosBagWithParameters
				;;
			'-i' )
				command_arguments="${command_arguments} state_estimation/pose"
				startRosBagWithParameters
				;;
			'-t' )
				command_arguments="${command_arguments} status/temperature electrical_interface/temperature electrical_interface/temperature1 electrical_interface/temperature2 electrical_interface/temperature3 electrical_interface/temperature4 electrical_interface/temperature5"
				startRosBagWithParameters
				;;
			'-c' )
				#down camera:
				command_arguments="${command_arguments} camera_down/camera_out/camera_info camera_down/camera_out/image_raw"
				#front camera
				command_arguments="${command_arguments} camera_front_left/camera_out/camera_info camera_front_left/camera_out/image_raw"
	            startRosBagWithParameters
	            ;;
			'-d' )
				command_arguments="${command_arguments} electrical_interface/depth"
				startRosBagWithParameters
				;;
			'-h' )
				usage
				;;
			* )
				echo "The input '${arg}' is not valid, try -h to see all the supported parameters."
	            ;;
		esac
	done
}

if [ "$#" -eq "0" ]
	then
		usage
		exit
	else
		handleInputParameters $@
fi
