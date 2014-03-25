################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/hydrophones/scripts/demo/adc.cpp 

OBJS += \
./catkin_ws/src/hydrophones/scripts/demo/adc.o 

CPP_DEPS += \
./catkin_ws/src/hydrophones/scripts/demo/adc.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/hydrophones/scripts/demo/%.o: ../catkin_ws/src/hydrophones/scripts/demo/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


