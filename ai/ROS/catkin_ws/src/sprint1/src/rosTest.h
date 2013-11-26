#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "sprint1/Depth.h"
#include "sprint1/Pressure.h"

#include <sstream>

#ifndef rosTest_h
#define rosTest_h



class rosTest {

 public:
     int main(int argc, char **argv);

     rosTest(double  x, double  y, double  depth);

     rosTest(double  x, double  y, double  depth, std::string  id);

 public:
    double x;
    double y;
    double depth;
    std::string Id;
};

#endif // rosTest_h

