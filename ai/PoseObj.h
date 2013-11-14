#ifndef PoseObj_h
#define PoseObj_h



class PoseObj {

 public:

     PoseObj(double  x, double  y, double  depth);

     PoseObj(double  x, double  y, double  depth, std::string  id);

 public:
    double x;
    double y;
    double depth;
    std::string Id;
};

#endif // PoseObj_h
