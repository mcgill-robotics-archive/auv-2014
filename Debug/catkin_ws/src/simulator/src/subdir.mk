################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/simulator/src/tf_broadcaster_simulator.cpp 

CC_SRCS += \
../catkin_ws/src/simulator/src/create_torpedo.cc \
../catkin_ws/src/simulator/src/launch_torpedo.cc \
../catkin_ws/src/simulator/src/move_model.cc \
../catkin_ws/src/simulator/src/robot.cc 

OBJS += \
./catkin_ws/src/simulator/src/create_torpedo.o \
./catkin_ws/src/simulator/src/launch_torpedo.o \
./catkin_ws/src/simulator/src/move_model.o \
./catkin_ws/src/simulator/src/robot.o \
./catkin_ws/src/simulator/src/tf_broadcaster_simulator.o 

CC_DEPS += \
./catkin_ws/src/simulator/src/create_torpedo.d \
./catkin_ws/src/simulator/src/launch_torpedo.d \
./catkin_ws/src/simulator/src/move_model.d \
./catkin_ws/src/simulator/src/robot.d 

CPP_DEPS += \
./catkin_ws/src/simulator/src/tf_broadcaster_simulator.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/simulator/src/%.o: ../catkin_ws/src/simulator/src/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '

catkin_ws/src/simulator/src/%.o: ../catkin_ws/src/simulator/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


