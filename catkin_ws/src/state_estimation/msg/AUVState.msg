#When CV loses sight of object, this will contain the best estimate
#of the distance to the last seen target.
#Currently, bearing (atan2(x,y)) will stay accurate but the length of the
#distance vector will probably drift, when no target is in sight
computer_vision/VisibleObjectData visibleObjectData
#Does the CV have an object in sight?
bool hasTarget

float64 depth

#TODO:
float64 pitch
float64 pitch_rate
float64 roll
float64 roll_rate
geometry_msgs/Vector3 velocity


