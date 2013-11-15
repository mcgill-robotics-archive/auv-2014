#include "ros/ros.h"
#include "std_msgs/Float64.h"

#ifndef ROS_h
#define ROS_h

#include "PoseObj.h"
#include "ROS.h"


class ROS {

 public:

    ROS();

    double GetDepth();

    double GetX();

    double GetY();

    PoseObj GetPose();
/*
    Vector Velocity();
*/
    double GetPressure();

    int main(int argc,char **argv);

 private:
    double Depth;
    double GlobalX;
    double GlobalY;
};

#endif // ROS_h
