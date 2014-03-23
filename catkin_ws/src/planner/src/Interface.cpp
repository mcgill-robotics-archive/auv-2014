#include "Interface.h"

//totally going to need to tweak these values at some point
double maxDepthError = 0.5;
double maxHeadingError = 0.5;

ros::Subscriber estimatedState_subscriber;
ros::Subscriber estimatedDepth_subscriber;

ros::Publisher wrench_pub;
ros::Publisher CV_objs_pub;
ros::Publisher control_pub;
ros::Publisher checkpoints_pub;
ros::Publisher taskPubFront;
ros::Publisher taskPubDown;

geometry_msgs::PoseStamped myPose;
geometry_msgs::PoseStamped relativePose;
/**
 * Our current orientation from state estimation
 */
double visible_XPos;
double visible_YPos;
double visible_Depth;
double visible_Yaw;
double visible_Pitch;

/**
 * Defines in which folder are stored the XML files.
*/
std::string xmlFilesPath;

/**
 * Object the we currently want CV to look for
 */
std::string visionObj;

void spinThread()
{
  ros::spin();
}

void estimatedState_callback(const computer_vision::VisibleObjectData data) {
  visible_XPos = data.x_distance;
  visible_YPos = data.y_distance;
  visible_Pitch = data.pitch_angle; 
  visible_Yaw = data.yaw_angle; 
}

void estimatedDepth_callback(const std_msgs::Float64 msg) {
  visible_Depth = msg.data;
}


/**
* determines the position and heading of our robot
*/
void setOurPose() {
  tf::TransformListener listener;
  geometry_msgs::PoseStamped emptyPose;
  emptyPose.header.frame_id = "/robot/rotation_center";
  emptyPose.pose.position.x = 0.0;
  emptyPose.pose.position.y = 0.0;
  emptyPose.pose.position.z = 0.0;
  emptyPose.pose.orientation.x = 0.0;
  emptyPose.pose.orientation.y = 0.0;
  emptyPose.pose.orientation.z = 0.0;
  emptyPose.pose.orientation.w = 1.0;
  listener.transformPose("/robot/rotation_center", emptyPose, myPose);
}

/**
* gets the position/heading of robot relative to chosen object
*/
void setTransform (std::string referenceFrame) {
  tf::TransformListener listener;
  setOurPose();
  listener.transformPose(referenceFrame, myPose, relativePose);
}

bool areWeThereYet(std::vector<double> desired) {
  //if (estimatedDepth_subscriber.getNumPublishers() == 0) {return false;}
  //if (estimatedState_subscriber.getNumPublishers() == 0) {return false;}
  double xError = visible_XPos - desired.at(0);
  double yError = visible_YPos - desired.at(1);
  double pitchError = visible_Pitch - desired.at(2);
  double yawError = visible_Yaw - desired.at(3);
  double depthError = visible_Depth - desired.at(4);

  /******below will show the errors in case of issues (comment out if needed)
  std::cout<<"xError: " << xError <<std::endl;
  std::cout<<"yError: " << yError <<std::endl;
  std::cout<< "pitchError: " << pitchError <<std::endl;
  std::cout<<"yawError: " << yawError <<std::endl;
  std::cout<<"depthError: " << depthError << "\n---------------------------------------\n\n"; */

  return (xError < .1 && yError < .1 && pitchError < .1 &&
          yawError < .1 && depthError < .1);
}


bool areWeThereYet_tf(std::string referenceFrame) {
  //if (estimatedDepth_subscriber.getNumPublishers() == 0) {return false;}
  //if (estimatedState_subscriber.getNumPublishers() == 0) {return false;}
  setTransform(referenceFrame);
  //positional bounds
  bool pxBounded = relativePose.pose.position.x < .1;
  bool pyBounded = relativePose.pose.position.y < .1;
  bool pzBounded = relativePose.pose.position.z < .1;
  //rotational bounds
  bool txBounded = relativePose.pose.orientation.x < .1;
  bool tyBounded = relativePose.pose.orientation.y < .1;
  bool tzBounded = relativePose.pose.orientation.z < .1;
  bool twBounded = relativePose.pose.orientation.w < .1;
  return (pxBounded && pyBounded && pzBounded && txBounded &&
      tyBounded && tzBounded && twBounded);
}

void setVisionObj (int objIndex) {
  planner::CurrentCVTask msgFront;
  planner::CurrentCVTask msgDown;

  msgFront.currentCVTask = msgFront.NOTHING;
  msgDown.currentCVTask = msgDown.NOTHING;
  switch (objIndex) {
    case 0 : msgFront.currentCVTask = msgFront.NOTHING;
             msgDown.currentCVTask = msgFront.NOTHING;
      break;
    case 1 : msgFront.currentCVTask = msgFront.GATE;
             msgDown.currentCVTask = msgFront.NOTHING;
      break;
    case 2 : msgDown.currentCVTask = msgFront.LANE;
             msgFront.currentCVTask = msgFront.NOTHING;
      break;
    case 3 : msgFront.currentCVTask = msgFront.BUOY;
             msgDown.currentCVTask = msgFront.NOTHING;
      break;
  }

  taskPubFront.publish(msgFront);
  taskPubDown.publish(msgDown);
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

void setPosition (std::vector<double> desired) {
  double pointControl[16] = {1, desired.at(0), 1, desired.at(1), 0, desired.at(2), 
    0, desired.at(3), 0, 0, 0, 0, 0, 0, 1, desired.at(4)};
  setPoints(pointControl);
}

void rosSleep(int length) {
  ros::Time time = ros::Time::now();
  ros::Duration d = ros::Duration(length, 0);
  d.sleep();
}

int main (int argc, char **argv) {
  ros::init(argc, argv, "Planner");
  ros::NodeHandle n;

  estimatedState_subscriber = n.subscribe("/front_cv_data", 1000, estimatedState_callback);
  estimatedDepth_subscriber = n.subscribe("depthCalculated", 1000, estimatedDepth_callback);

  taskPubFront = n.advertise<planner::CurrentCVTask>("current_cv_task_front", 1000); 
  taskPubDown = n.advertise<planner::CurrentCVTask>("current_cv_task_down", 1000); 
  checkpoints_pub = n.advertise<std_msgs::String>("planner/task", 1000);
  control_pub = n.advertise<planner::setPoints>("setPoints", 1000);

  n.param<std::string>("Planner/xml_files_path", xmlFilesPath, "");
  
  ROS_INFO("%s", ("Value of xmlFilesPath=" + xmlFilesPath).c_str());

  bool ready = 0;
  while (ready == 0)
  {
    ROS_DEBUG_THROTTLE(2,"Waiting...");
    ready = 1;     
    if (estimatedDepth_subscriber.getNumPublishers() == 0) {ready = 0;}
    else {ROS_DEBUG_THROTTLE(2,"Here ye Heare ye");}
    if (estimatedState_subscriber.getNumPublishers() == 0) {ready = 0;}
    else {ROS_DEBUG_THROTTLE(2,"got estimated State");}
  }

  ros::Rate loop_rate(10);
  /****This is ros::spin() on a seperate thread*****/
  boost::thread spin_thread(&spinThread);

  std::cout<< "Starting Loader" << std::endl; 
  Loader* loader = new Loader(xmlFilesPath);
  Invoker* invoker = loader->getInvoker();
  invoker->StartRun();
  std::cout<<"Done Loader"<< std::endl;  

  delete loader; delete invoker;


  return 0;
}

//LEGACY -- NOT FOR TOUCHING
void ps3Control () {}
