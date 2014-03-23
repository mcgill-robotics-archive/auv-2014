################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/sensor_publisher/src/XimuPublisher.cpp \
../catkin_ws/src/sensor_publisher/src/XimuReceiver.cpp \
../catkin_ws/src/sensor_publisher/src/madgwick.cpp 

C_SRCS += \
../catkin_ws/src/sensor_publisher/src/MadgwickAHRS.c 

OBJS += \
./catkin_ws/src/sensor_publisher/src/MadgwickAHRS.o \
./catkin_ws/src/sensor_publisher/src/XimuPublisher.o \
./catkin_ws/src/sensor_publisher/src/XimuReceiver.o \
./catkin_ws/src/sensor_publisher/src/madgwick.o 

C_DEPS += \
./catkin_ws/src/sensor_publisher/src/MadgwickAHRS.d 

CPP_DEPS += \
./catkin_ws/src/sensor_publisher/src/XimuPublisher.d \
./catkin_ws/src/sensor_publisher/src/XimuReceiver.d \
./catkin_ws/src/sensor_publisher/src/madgwick.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/sensor_publisher/src/%.o: ../catkin_ws/src/sensor_publisher/src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C Compiler'
	gcc -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

catkin_ws/src/sensor_publisher/src/%.o: ../catkin_ws/src/sensor_publisher/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


