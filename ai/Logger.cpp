#include <iostream>
#include <fstream>
#include <string>
#include <ctime> //for getTime method
#include <sys/stat.h> //for make dir1
#include <sstream> //for int to string conversion
using namespace std;

//directory storing ai log file
const char AI_LOG_DIR[] = "./ai_log/";

using namespace std;

//STUB POSEOBJ FOR TESTING
class PoseObj{
public:
  PoseObj(int  x, int  y, int  depth, string  id);
  double x;
  double y;
  double depth;
  std::string Id;
};

PoseObj::PoseObj(int X , int Y , int Dep,string ID ){
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
};

/**
 * returns a string that contains current time:
 * @param : hasSec : if true then append second to the end :
 */
string getTime(bool hasSec){
	  // system time
	  std::time_t rawtime;
	  std::tm* timeinfo;
	  std::time(&rawtime);
	  char currentTime [80];
	  string timeFormat = string("%Y_%m_%d_%H_%M").append  (hasSec ? "_%S" : "" ) ;

	  //get current time
	  timeinfo = std::localtime(&rawtime);
	  strftime(currentTime,80,timeFormat.c_str(),timeinfo);
	  return ( currentTime ) ;
}

//constructor
Logger::Logger()
{
	string currentTime = getTime(false);	//get current time

  //make directory
  if (mkdir(AI_LOG_DIR, S_IRWXU|S_IRGRP|S_IXGRP) != 0)
    {
      perror("mkdir() error");
    }
  else
    {
      cout << "successfully made dir : " + string(AI_LOG_DIR) <<"\n" ;
    }

  //create a new file in the directory
  //file name is yyy-mm-dd-hh-mm.txt in ai_log dir
  fileName = AI_LOG_DIR + currentTime + ".txt" ;
  cout << "made file " + fileName +"\n";

}

int Logger::writeString (string s)
{
	ofstream out ;
	out.open(this->fileName.c_str(),fstream::app); //open file and append

	//write to file
	//mark down the time stamp
	out << "[" + getTime(true) +"] \t" <<  s  << "\n";
	out.close();
  return 0;
}


int Logger::writePose (PoseObj p)
{
	//walk round for the to_string bug
    std::ostringstream ss;

    ss << p.x ;
    string px = ss.str();
    ss << p.y ;
    string py = ss.str();
    ss << p.depth ;
    string pDepth = ss.str();


	writeString("Id: " + p.Id + "\t"+
			"position x: " + px +"\t"
			+"position y: " + py + "\t" +
			"depth : " + pDepth
			);

  return 0 ;
}


int main()
{

  Logger l;
  PoseObj p(1,212,3,"hellowe") ;
  cout << l.fileName;
  l.writeString("cool");
  l.writePose (p);
  cout <<getTime(true) << "\n";
  cout <<getTime(false) << "\n";



  return 0;
}
