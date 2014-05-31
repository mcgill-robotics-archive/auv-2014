#!/bin/bash
############################
# This script allows a used to ROSBAG specific, or all, topics of certain components very easily.
############################

# Variable initialization
command_arguments=''

# Displays the usage of this script.
usage() {
    echo USAGE: rosbagHelper.sh OPTION
    echo "options: -a : all topics specified under"  
    echo "         -i : imu pose and raw data"
    echo "         -t : data from all temperature sensors"
    echo "         -c : camera data; i.e. camera info, image rectified and color image from up and down cameras"
    echo "         -d : depth sensor data"
    echo "         -v : voltages from both batteries"
    echo "         -e : motor message, usb, solenoid, and pressure inside main pressure vessel" 
    echo "         -h : prints this menu"
    exit
}

# Starts ROSBag with the given parameters.
startRosBagWithParameters() {
	rosbag record $command_arguments -O ~/bag-$(date +%s)
}

addImuTopics() {
	command_arguments="${command_arguments} state_estimation/pose state_estimation/raw"
}

addTemperatureTopics() {
	command_arguments="${command_arguments} electrical_interface/batteryVoltage1 electrical_interface/batteryVoltage2"
}

addCameraTopics() {
	command_arguments="${command_arguments} camera_down/camera_out/camera_info camera_down/image_rect camera_front_left/camera_out/camera_info camera_front_left/image_rect"
}

addDepthTopic() {
	command_arguments="${command_arguments} electrical_interface/depth"
}

addVoltageTopics() {
	echo ""
}

addElectricalInterfaceTopics() {
	command_arguments="${command_arguments} status/usb electrical_interface/pressure electrical_interface/motor electrical_interface/solenoid"
}

# Handles the input of the scripts.
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
				;;
			'-i' )
				command_arguments="${command_arguments} state_estimation/pose"
				;;
			'-t' )
				command_arguments="${command_arguments} status/temperature electrical_interface/temperature electrical_interface/temperature1 electrical_interface/temperature2 electrical_interface/temperature3 electrical_interface/temperature4 electrical_interface/temperature5"
				;;
			'-c' )
				#down camera:
				command_arguments="${command_arguments} camera_down/camera_out/camera_info camera_down/camera_out/image_raw"
				#front camera
				command_arguments="${command_arguments} camera_front_left/camera_out/camera_info camera_front_left/camera_out/image_raw"
	            ;;
			'-d' )
				command_arguments="${command_arguments} electrical_interface/depth"
				;;
			'-h' )
				usage
				;;
			* )
				echo "The input '${arg}' is not valid, try -h to see all the supported parameters."
	            ;;
		esac
	done
	startRosBagWithParameters
}

if [ "$#" -eq "0" ]
	then
		usage
		exit
	else
		handleInputParameters $@
fi
