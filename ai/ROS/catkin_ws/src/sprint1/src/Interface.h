#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sprint1/Depth.h"
#include "sprint1/Pressure.h"
#include "sprint1/CVQuery.h"

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
