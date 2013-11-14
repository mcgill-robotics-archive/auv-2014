#ifndef Logger_h
#define Logger_h

#include <iostream>
#include <fstream>
#include <string>
#include <ctime> //for getTime method
#include <sys/stat.h> //for make dir1
#include <sstream> //for int to string conversion
using namespace std;

//directory storing ai log file
const char AI_LOG_DIR[] = "./ai_log/";


//STUB POSEOBJ FOR TESTING
class PoseObj{
public:
  PoseObj(double  x, double  y, double  depth, string  id);
  double x;
  double y;
  double depth;
  std::string Id;
};

PoseObj::PoseObj(double X , double Y , double Dep,string ID ){
  x = X;
  y = Y;
  depth = Dep ;
  Id = ID;
}


class Logger {

public :
  Logger ();
  int writeString (string s);
  int writePose (PoseObj p);
  string fileName;
//  int WriteLogConfig(void  Loader);

};


#endif // Logger_h
