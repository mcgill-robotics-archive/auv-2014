################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CC_SRCS += \
../catkin_ws/src/simulator_controls_wrench_test/src/controls_wrench_test.cc 

OBJS += \
./catkin_ws/src/simulator_controls_wrench_test/src/controls_wrench_test.o 

CC_DEPS += \
./catkin_ws/src/simulator_controls_wrench_test/src/controls_wrench_test.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/simulator_controls_wrench_test/src/%.o: ../catkin_ws/src/simulator_controls_wrench_test/src/%.cc
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


