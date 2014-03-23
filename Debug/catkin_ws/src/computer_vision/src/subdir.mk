################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/computer_vision/src/Buoy.cpp \
../catkin_ws/src/computer_vision/src/CVNode.cpp \
../catkin_ws/src/computer_vision/src/CVTestNode.cpp \
../catkin_ws/src/computer_vision/src/Camera.cpp \
../catkin_ws/src/computer_vision/src/CameraNode.cpp \
../catkin_ws/src/computer_vision/src/Door.cpp \
../catkin_ws/src/computer_vision/src/DownCVNode.cpp \
../catkin_ws/src/computer_vision/src/DownCameraNode.cpp \
../catkin_ws/src/computer_vision/src/FrontCVNode.cpp \
../catkin_ws/src/computer_vision/src/FrontCameraNode.cpp \
../catkin_ws/src/computer_vision/src/LineTarget.cpp \
../catkin_ws/src/computer_vision/src/MarkerTarget.cpp \
../catkin_ws/src/computer_vision/src/SoftCameraNode.cpp \
../catkin_ws/src/computer_vision/src/VisibleObject.cpp 

OBJS += \
./catkin_ws/src/computer_vision/src/Buoy.o \
./catkin_ws/src/computer_vision/src/CVNode.o \
./catkin_ws/src/computer_vision/src/CVTestNode.o \
./catkin_ws/src/computer_vision/src/Camera.o \
./catkin_ws/src/computer_vision/src/CameraNode.o \
./catkin_ws/src/computer_vision/src/Door.o \
./catkin_ws/src/computer_vision/src/DownCVNode.o \
./catkin_ws/src/computer_vision/src/DownCameraNode.o \
./catkin_ws/src/computer_vision/src/FrontCVNode.o \
./catkin_ws/src/computer_vision/src/FrontCameraNode.o \
./catkin_ws/src/computer_vision/src/LineTarget.o \
./catkin_ws/src/computer_vision/src/MarkerTarget.o \
./catkin_ws/src/computer_vision/src/SoftCameraNode.o \
./catkin_ws/src/computer_vision/src/VisibleObject.o 

CPP_DEPS += \
./catkin_ws/src/computer_vision/src/Buoy.d \
./catkin_ws/src/computer_vision/src/CVNode.d \
./catkin_ws/src/computer_vision/src/CVTestNode.d \
./catkin_ws/src/computer_vision/src/Camera.d \
./catkin_ws/src/computer_vision/src/CameraNode.d \
./catkin_ws/src/computer_vision/src/Door.d \
./catkin_ws/src/computer_vision/src/DownCVNode.d \
./catkin_ws/src/computer_vision/src/DownCameraNode.d \
./catkin_ws/src/computer_vision/src/FrontCVNode.d \
./catkin_ws/src/computer_vision/src/FrontCameraNode.d \
./catkin_ws/src/computer_vision/src/LineTarget.d \
./catkin_ws/src/computer_vision/src/MarkerTarget.d \
./catkin_ws/src/computer_vision/src/SoftCameraNode.d \
./catkin_ws/src/computer_vision/src/VisibleObject.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/computer_vision/src/%.o: ../catkin_ws/src/computer_vision/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


