#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Int64.h"
#include "std_msgs/String.h"
#include "planner/MotorControl.h"

#ifndef Interface_h
#define Interface_h

#include "Interface.h"


class Interface {

 public:

    Interface();
    
    void setDepth();

    double getDepth();

    void setPosition();

    double GetX();

    double GetY();

//    PoseObj GetPose();

//    Vector Velocity();

    void setPressure();

    double getPressure();

    int main(int argc,char **argv);

 private:
    double Depth;
};

#endif // Interface_h
