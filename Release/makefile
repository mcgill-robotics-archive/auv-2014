################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include catkin_ws/src/simulator_thruster_test/src/subdir.mk
-include catkin_ws/src/simulator_controls_wrench_test/src/subdir.mk
-include catkin_ws/src/simulator/src/subdir.mk
-include catkin_ws/src/sensor_publisher/src/subdir.mk
-include catkin_ws/src/robot_pose_ekf/test/subdir.mk
-include catkin_ws/src/robot_pose_ekf/src/subdir.mk
-include catkin_ws/src/pose_ukf/test/subdir.mk
-include catkin_ws/src/pose_ukf/src/subdir.mk
-include catkin_ws/src/planner/src/subdir.mk
-include catkin_ws/src/planner/include/pugixml/src/subdir.mk
-include catkin_ws/src/planner/include/pugixml/docs/samples/subdir.mk
-include catkin_ws/src/mock_data/src/subdir.mk
-include catkin_ws/src/hydrophones/scripts/demo/subdir.mk
-include catkin_ws/src/controls/src/subdir.mk
-include catkin_ws/src/computer_vision/src/subdir.mk
-include Arduino/teensy/subdir.mk
-include Arduino/libraries/ros_lib/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C++_DEPS)),)
-include $(C++_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
ifneq ($(strip $(CC_DEPS)),)
-include $(CC_DEPS)
endif
ifneq ($(strip $(CPP_DEPS)),)
-include $(CPP_DEPS)
endif
ifneq ($(strip $(CXX_DEPS)),)
-include $(CXX_DEPS)
endif
ifneq ($(strip $(C_UPPER_DEPS)),)
-include $(C_UPPER_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 

# All Target
all: McGill_RoboSub_2014

# Tool invocations
McGill_RoboSub_2014: $(OBJS) $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: GCC C++ Linker'
	g++  -o "McGill_RoboSub_2014" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(OBJS)$(C++_DEPS)$(C_DEPS)$(CC_DEPS)$(CPP_DEPS)$(EXECUTABLES)$(CXX_DEPS)$(C_UPPER_DEPS) McGill_RoboSub_2014
	-@echo ' '

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
