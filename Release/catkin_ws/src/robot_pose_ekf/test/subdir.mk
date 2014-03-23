################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf.cpp \
../catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf_zero_covariance.cpp 

OBJS += \
./catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf.o \
./catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf_zero_covariance.o 

CPP_DEPS += \
./catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf.d \
./catkin_ws/src/robot_pose_ekf/test/test_robot_pose_ekf_zero_covariance.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/robot_pose_ekf/test/%.o: ../catkin_ws/src/robot_pose_ekf/test/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


