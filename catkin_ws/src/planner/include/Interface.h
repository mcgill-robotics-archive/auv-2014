#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "std_msgs/String.h"
#include "std_msgs/Int64.h"
#include "simulator/ThrusterForces.h"
#include "planner/setPoints.h"

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

    void setPoints(double pointControl[]);

    void setPosition(double x_pos, double y_pos, double z_pos, double pitch_angle, double yaw_angle);

    void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth);

    int main(int argc,char **argv);
};

#endif // Interface_h
