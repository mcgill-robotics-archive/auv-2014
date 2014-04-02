# This custom message type is to be used to debug the controls
# It contains values published by controls that should be useful
# for debugging the control system

# Errors

VectorPID xError
VectorPID yError
VectorPID depthError
VectorPID pitchError
VectorPID yawError

# Gains

VectorPID xGain
VectorPID yGain
VectorPID depthGain
VectorPID pitchGain
VectorPID yawGain

# PID Forces (and Torques)

VectorPID xForce
VectorPID yForce
VectorPID depthForce
VectorPID pitchForce
VectorPID yawForce

# Total Forces?

# Thrusts
int32 thrust_surge_port
int32 thrust_surge_starboard
int32 thrust_sway_bow
int32 thrust_sway_stern
int32 thrust_heave_bow
int32 thrust_heave_stern

