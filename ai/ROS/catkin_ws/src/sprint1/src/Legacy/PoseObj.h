#include "ros/ros.h"
#include "std_msgs/Float64.h"

#include <sstream>

#ifndef PoseObj_h
#define PoseObj_h



class PoseObj {

 public:
     int main(int argc, char **argv);

     PoseObj(double  x, double  y, double  depth);

     PoseObj(double  x, double  y, double  depth, std::string  id);

 public:
    double x;
    double y;
    double depth;
    std::string Id;
};

#endif // PoseObj_h

