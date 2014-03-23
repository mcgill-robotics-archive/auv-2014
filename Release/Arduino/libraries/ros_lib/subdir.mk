################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Arduino/libraries/ros_lib/duration.cpp \
../Arduino/libraries/ros_lib/time.cpp 

OBJS += \
./Arduino/libraries/ros_lib/duration.o \
./Arduino/libraries/ros_lib/time.o 

CPP_DEPS += \
./Arduino/libraries/ros_lib/duration.d \
./Arduino/libraries/ros_lib/time.d 


# Each subdirectory must supply rules for building sources it contributes
Arduino/libraries/ros_lib/%.o: ../Arduino/libraries/ros_lib/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


