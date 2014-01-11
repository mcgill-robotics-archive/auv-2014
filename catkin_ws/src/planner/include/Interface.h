#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "simulator/ThrusterForces.h"

#ifndef Interface_h
#define Interface_h

#include "Interface.h"

class Interface {
  public:
    Interface();
    
    double getDepth();
    void setDepth(const std_msgs::Float64::ConstPtr& msg);

    double getPressure();
    void setPressure(const std_msgs::Float64::ConstPtr& msg);

    void TODO(const geometry_msgs::Twist msg);

    void setOrientation(const geometry_msgs::Quaternion msg);

    int main(int argc,char **argv);
};

#endif // Interface_h
