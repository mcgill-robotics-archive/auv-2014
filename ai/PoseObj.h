#ifndef PoseObj_h
#define PoseObj_h



class PoseObj {

 public:

    virtual  PoseObj(void  x, void  y, void  depth);

    virtual  PoseObj(void  x, void  y, void  depth, void  id);

 public:
    double x;
    double y;
    double depth;
    string Id;
};

#endif // PoseObj_h
