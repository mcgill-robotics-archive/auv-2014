#!/bin/bash
############################
# This script allows a used to ROSBAG specific, or all, topics of certain components very easily.
############################

# Variable initialization
command_arguments=''
nameOfRosBag=bag-$(date +%s)
targetDirectory=~/
checkingForFileName=false

# Displays the usage of this script.
usage() {
    echo USAGE: rosbagHelper.sh OPTION
    echo "options: -a : all topics specified below except for hydrophones"
    echo "         -i : imu pose and raw data"
    echo "         -t : data from all temperature sensors"
    echo "         -c : camera data; i.e. camera info, image rectified and color image from up and down cameras"
    echo "         -d : depth sensor data"
    echo "         -v : voltages from both batteries"
    echo "         -e : motor message, usb, solenoid, and pressure inside main pressure vessel"
    echo "         -h : all hydrophones topics"
    echo "         -s : all state estimation topics (including IMU specific topics)"
    echo "         -help : prints this menu"
    echo "         -name nameOfYourROSBag : Will save the ROSbag using 'nameOfYourROSBag' as the filename."
    echo "                                  If the parameter is not present, a timestamp will be used."
    exit
}

# Starts ROSBag with the given parameters.
startRosBagWithParameters() {
	rosbag record $command_arguments -O ${targetDirectory}${nameOfRosBag}
}

addImuTopics() {
	command_arguments="${command_arguments} state_estimation/pose state_estimation/raw"
}

addTemperatureTopics() {
	command_arguments="${command_arguments} electrical_interface/batteryVoltage1 electrical_interface/batteryVoltage2"
}

addCameraTopics() {
	command_arguments="${command_arguments} camera_down/camera_out/camera_info camera_down/image_rect"
	command_arguments="${command_arguments} camera_front_left/camera_out/camera_info camera_front_left/image_rect"
}

addDepthTopic() {
	command_arguments="${command_arguments} electrical_interface/depth"
}

addVoltageTopics() {
	command_arguments="${command_arguments} electrical_interface/batteryVoltage1 electrical_interface/batteryVoltage2"
}

addElectricalInterfaceTopics() {
	command_arguments="${command_arguments} status/usb electrical_interface/pressure electrical_interface/motor electrical_interface/solenoid"
}

addStateEstimationTopics() {
	# TODO: There might be other topics that we want to add.
	addImuTopics
	command_arguments="${command_arguments} tf"
}

addHydrophonesTopics() {
	command_arguments="${command_arguments} hydrophones/audio hydrophones/freq hydrophones/magn hydrophones/peak hydrophones/sol hydrophones/tdoa"
}

# Changes the name of the ROS bag to what the user specified. Does some check on the name and will use a timestamp if rejected.
changeNameOfRosBag() {
	if [ -e "${targetDirectory}$1" ]; then
		echo "The filename you specified for the ROSBag is in conflict with an existing file. A timestamp will be used instead."
	else
		nameOfRosBag=$1
		#echo "Using '${nameOfRosBag}' as filename."
	fi
	checkingForFileName=false
}

# Handles the input of the scripts.
handleInputParameters() {
	for arg in "$@"; do

		if [ $checkingForFileName = false ]
			then
				case ${arg} in
					'-a' )
						addImuTopics
						addTemperatureTopics
						addCameraTopics
						addDepthTopic
						addVoltageTopics
						addElectricalInterfaceTopics
						# addHydrophonesTopics
						;;
					'-i' )
						addImuTopics
						;;
					'-t' )
						addTemperatureTopics
						;;
					'-c' )
						addCameraTopics
						;;
					'-d' )
						addDepthTopic
						;;
					'-h' )
						addHydrophonesTopics
						addImuTopics
						;;
					'-v' )
						addVoltageTopics
						;;
					'-e' )
						addElectricalInterfaceTopics
						;;
					'-help' )
						usage
						;;
					'-name' )
						checkingForFileName=true
						continue # This is to make sure that on the next iteration I will be checking for the file name.
						;;
					* )
						echo "The input '${arg}' is not valid, try -help to see all the supported parameters."
			            ;;
				esac
			else
				changeNameOfRosBag ${arg}
		fi
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
