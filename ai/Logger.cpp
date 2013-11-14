#include "Logger.h"



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
      perror("mkdir() error, directory already exist ");
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

    ss.str("");	//empty stream
    ss << p.x ;
    string px = ss.str();
    ss.str("");
    ss << p.y ;
    string py = ss.str();
    ss.str("");
    ss << p.depth ;
    string pDepth = ss.str();

	writeString("Id: " + p.Id + "\t"+
			"position x: " + px +"\t"
			+"position y: " + py + "\t" +
			"depth : " + pDepth
			);

  return 0 ;
}

//
//int main()
//{
//
//  Logger l;
//  PoseObj p(3.14,3.14*2,3.14*3,"String ID") ;
//  cout << l.fileName;
//  l.writeString("cool");
//  l.writePose (p);
//  cout <<getTime(true) << "\n";
//  cout <<getTime(false) << "\n";
//
//
//  return 0;
//}
