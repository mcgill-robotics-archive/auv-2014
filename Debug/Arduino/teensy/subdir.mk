################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
CPP_SRCS += \
../Arduino/teensy/button.cpp \
../Arduino/teensy/hydrophones.cpp \
../Arduino/teensy/mics.cpp 

OBJS += \
./Arduino/teensy/button.o \
./Arduino/teensy/hydrophones.o \
./Arduino/teensy/mics.o 

CPP_DEPS += \
./Arduino/teensy/button.d \
./Arduino/teensy/hydrophones.d \
./Arduino/teensy/mics.d 


# Each subdirectory must supply rules for building sources it contributes
Arduino/teensy/%.o: ../Arduino/teensy/%.cpp
	@echo 'Building file: $<'
	@echo 'Invoking: GCC C++ Compiler'
	g++ -I/opt/ros/hydro/include/opencv -O0 -g3 -Wall -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@:%.o=%.d)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


