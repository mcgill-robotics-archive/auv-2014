#include "Interface.h"

ros::Publisher wrench_pub;
ros::Publisher CV_objs_pub;
ros::Publisher control_pub;
ros::Publisher checkpoints_pub;

/**
 * Our current orientation from state estimation
 */
double Self_XDistance;
double Self_YDistance;
double Self_ZDistance;
double Self_Yaw;
double Self_Pitch;

/**
 * Object the we currently want CV to look for
 */
std::string visionObj;

/**
 * Orientation of the object currently in view of CV
 */
double VO_XDistance;
double VO_YDistance;
double VO_ZDistance;
double VO_Yaw;
double VO_Pitch;

void setVisibleObjectOrientation (computer_vision::VisibleObjectData msg) {
  VO_XDistance = msg.x_distance;
  VO_YDistance = msg.y_distance;
  VO_ZDistance = msg.z_distance;
  VO_Yaw = msg.yaw_angle;
  VO_Pitch = msg.pitch_angle;
}

//BEWARE: RETURNS ADDRESS OF ARRAY
double* getVisibleObjectOrientation () {
  double returnArray[5];
  returnArray[0] = VO_XDistance;
  returnArray[1] = VO_YDistance;
  returnArray[2] = VO_ZDistance;
  returnArray[3] = VO_Yaw;
  returnArray[4] = VO_Pitch;
  return returnArray;
}

void setOurOrientation(gazebo_msgs::ModelStates msg) {
  Self_XDistance = msg.pose[0].position.x;
  Self_YDistance = msg.pose[0].position.y;
  Self_ZDistance = msg.pose[0].position.z;
  Self_Yaw = 0;
  Self_Pitch = 0;
}

//BEWARE: RETURNS ADDRESS OF ARRAY
double* getOurOrientation () {
  double returnArray[5];
  returnArray[0] = Self_XDistance;
  returnArray[1] = Self_YDistance;
  returnArray[2] = Self_ZDistance;
  returnArray[3] = Self_Yaw;
  returnArray[4] = Self_Pitch;
  return returnArray;
}

void setVisionObj (std::string obj) {
  visionObj = obj;
  std_msgs::String msgCV;
  msgCV.data = visionObj;
  CV_objs_pub.publish(msgCV);
}

void weAreHere (std::string task) {
  std_msgs::String msg;
  msg.data = task;
  checkpoints_pub.publish(msg);
}

void setPoints (double pointControl[]) {
  planner::setPoints msgControl;

  msgControl.XPos.isActive = pointControl[0];
  msgControl.XPos.data = pointControl[1];;
  
  msgControl.YPos.isActive = pointControl[2];
  msgControl.YPos.data = pointControl[3];
  
  msgControl.Yaw.isActive = pointControl[4];
  msgControl.Yaw.data = pointControl[5];

  msgControl.Pitch.isActive = pointControl[6];
  msgControl.Pitch.data = pointControl[7];
  
  msgControl.XSpeed.isActive = pointControl[8];
  msgControl.XSpeed.data = pointControl[9];

  msgControl.YSpeed.isActive = pointControl[10];
  msgControl.YSpeed.data = pointControl[11];

  msgControl.YawSpeed.isActive = pointControl[12];
  msgControl.YawSpeed.data = pointControl[13];

  msgControl.Depth.isActive = pointControl[14];
  msgControl.Depth.data = pointControl[15];

  control_pub.publish(msgControl);
}

void setVelocity (double x_speed, double y_speed, double yaw_speed, double depth) {
  double pointControl[16] = {0, 0, 0, 0, 0, 0, 0, 0,
    1, x_speed, 1, y_speed, 1, yaw_speed, 1,depth};
  setPoints(pointControl);
}

void setPosition (double x_pos, double y_pos, double pitch_angle, double yaw_angle, double depth) {
  double pointControl[16] = {1, x_pos, 1, y_pos, 1, pitch_angle, 
    1, yaw_angle, 0, 0, 0, 0, 0, 0, 1, depth};
  setPoints(pointControl);
}

//LEGACY -- NOT FOR TOUCHING
void ps3Control () {}

int main (int argc, char **argv) {
  ros::init(argc, argv, "Planner");
  ros::NodeHandle n;

  ros::Subscriber CV_sub = n.subscribe("visible_data", 1000, setVisibleObjectOrientation);
  ros::Subscriber Pose_sub = n.subscribe("gazebo/model_states", 1000, setOurOrientation);

  CV_objs_pub = n.advertise<std_msgs::String>("planner/CV_Object", 1000); 
  checkpoints_pub = n.advertise<std_msgs::String>("planner/task", 1000);
  control_pub = n.advertise<planner::setPoints>("setPoints", 1000);

  std::cout<<"Starting Loader"<< std::endl; 
  Loader* loader = new Loader();
  Invoker* invoker = loader->getInvoker();
  invoker->StartRun();
  std::cout<<"Done Loader"<< std::endl;  

  ros::Rate loop_rate(10);
  while ( ros::ok() ) {}
  ros::spin();
  return 0;
}

