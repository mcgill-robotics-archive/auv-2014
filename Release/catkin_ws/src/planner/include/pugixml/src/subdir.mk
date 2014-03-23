################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../catkin_ws/src/planner/include/pugixml/src/pugixml.cpp 

OBJS += \
./catkin_ws/src/planner/include/pugixml/src/pugixml.o 

CPP_DEPS += \
./catkin_ws/src/planner/include/pugixml/src/pugixml.d 


# Each subdirectory must supply rules for building sources it contributes
catkin_ws/src/planner/include/pugixml/src/%.o: ../catkin_ws/src/planner/include/pugixml/src/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -O3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


