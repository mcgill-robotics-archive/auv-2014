################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/mock_data/src/depthPublisher.cpp \
../catkin_ws/src/mock_data/src/posePublisher.cpp \
../catkin_ws/src/mock_data/src/pressurePublisher.cpp 

OBJS += \
./catkin_ws/src/mock_data/src/depthPublisher.o \
./catkin_ws/src/mock_data/src/posePublisher.o \
./catkin_ws/src/mock_data/src/pressurePublisher.o 

CPP_DEPS += \
./catkin_ws/src/mock_data/src/depthPublisher.d \
./catkin_ws/src/mock_data/src/posePublisher.d \
./catkin_ws/src/mock_data/src/pressurePublisher.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/mock_data/src/%.o: ../catkin_ws/src/mock_data/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


