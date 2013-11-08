#ifndef ROS_h
#define ROS_h

#include "PoseObj.h"
#include "ROS.h"


class ROS {

 public:

	virtual ROS();

    virtual double GetDepth();

    virtual double GetX();

    virtual double GetY();

    virtual PoseObj GetPose();

    virtual Vector Velocity();

    virtual double GetPressure();


 private:
    double Depth;
    double GlobalX;
    double GlobalY;
};

#endif // ROS_h
