################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/controls/src/controls.cpp \
../catkin_ws/src/controls/src/depthController.cpp \
../catkin_ws/src/controls/src/gazeboDepthEstimator.cpp \
../catkin_ws/src/controls/src/testControls.cpp \
../catkin_ws/src/controls/src/thrust_mapper.cpp 

OBJS += \
./catkin_ws/src/controls/src/controls.o \
./catkin_ws/src/controls/src/depthController.o \
./catkin_ws/src/controls/src/gazeboDepthEstimator.o \
./catkin_ws/src/controls/src/testControls.o \
./catkin_ws/src/controls/src/thrust_mapper.o 

CPP_DEPS += \
./catkin_ws/src/controls/src/controls.d \
./catkin_ws/src/controls/src/depthController.d \
./catkin_ws/src/controls/src/gazeboDepthEstimator.d \
./catkin_ws/src/controls/src/testControls.d \
./catkin_ws/src/controls/src/thrust_mapper.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/controls/src/%.o: ../catkin_ws/src/controls/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


