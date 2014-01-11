#include "Interface.h"

ros::Publisher ps3_pub;
ros::Publisher CV_objs_pub;
ros::Publisher control_pub;
/**
 * Object the we currently want CV to look for
 */
std::string visionObj;

/**
 * the depth of the robot as read from the sensors
 */
double depth;
/**
 * the pressure of the robot as read from the sensors
 */
double pressure;

/**
 * The orientation coordinates of the robot as read
 * from the sensors
 */
double orientX;
double orientY;
double orientZ;
double orientW;

/**
 * Variables for determining which points the motors
 * should ignore
 */
int XPosControl;
int YPosControl;
int ZPosControl;
int YawControl;
int PitchControl;
int XSpeedControl;
int YSpeedControl;
int YawSpeedControl;

/**
 * Variables containing the desired position of the
 * robot to tell motor control
 */
double XPos;
double YPos;
double ZPos;
double Yaw;
double Pitch;
double XSpeed;
double YSpeed;
double YawSpeed;

/**
 * Unused constructor
 */
Interface::Interface() {}

/**
 * Returns the current depth of the robot
 *
 * @return   the depth of the robot
 */
double getDepth() {
  return depth;
}

void setDepth(const std_msgs::Float64::ConstPtr& msg) {
  depth = msg->data;
}

double getPressure() {
  return pressure;
}

void setPressure(const std_msgs::Float64::ConstPtr& msg) {
  pressure = msg->data;
}

void TODO(const geometry_msgs::Twist msg) {}

void setOrientation(const geometry_msgs::Quaternion msg) {
  orientX = msg.x;
  orientY = msg.y;
  orientZ = msg.z;
  orientW = msg.w;
}

void setVisionObj(std::string obj) {
  visionObj = obj;
  std_msgs::String msgCV;
  msgCV.data = visionObj;
  CV_objs_pub.publish(msgCV);
}

void setPoints(int pointControl[]) {
  planner::setPoints msgControl;

  XPosControl = pointControl[0];
  XPos = pointControl[1];
  msgControl.XPos.isActive = XPosControl;
  msgControl.XPos.data = XPos;
  
  YPosControl = pointControl[2];
  YPos = pointControl[3];
  msgControl.YPos.isActive = YPosControl;
  msgControl.YPos.data = YPos;
  
  ZPosControl = pointControl[4];
  ZPos = pointControl[5];
  msgControl.ZPos.isActive = ZPosControl;
  msgControl.ZPos.data = ZPos;
  
  YawControl = pointControl[6];
  Yaw = pointControl[7];
  msgControl.Yaw.isActive = YawControl;
  msgControl.Yaw.data = Yaw;
  
  PitchControl = pointControl[8];
  Pitch = pointControl[9];
  msgControl.Pitch.isActive = PitchControl;
  msgControl.Pitch.data = Pitch;
  
  XSpeedControl = pointControl[10];
  XSpeed = pointControl[11];
  msgControl.XSpeed.isActive = XSpeedControl;
  msgControl.XSpeed.data = XSpeed;

  YSpeedControl = pointControl[12];
  YSpeed = pointControl[13];
  msgControl.YSpeed.isActive = YSpeedControl;
  msgControl.YSpeed.data = YSpeed;

  YawSpeedControl = pointControl[14];
  YawSpeed = pointControl[15];
  msgControl.YawSpeed.isActive = YawSpeedControl;
  msgControl.YawSpeed.data = YawSpeed;

  control_pub.publish(msgControl);
}

void ps3Control() {
  simulator::ThrusterForces msgPS3;
    msgPS3.ty1 = 1.0;
    msgPS3.ty2 = 1.0;
    msgPS3.tx1 = 0.0;
    msgPS3.tz1 = 0.0;
    msgPS3.tx2 = 0.0;
    msgPS3.tz2 = 0.0;

  ps3_pub.publish(msgPS3);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Planner");
  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth_data", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure_data", 1000, setPressure);
  ros::Subscriber IMU_sub = n.subscribe("imu_data", 1000, setOrientation);

  ps3_pub = n.advertise<simulator::ThrusterForces>("gazebo/thruster_forces", 1000);
  CV_objs_pub = n.advertise<std_msgs::String>("planner/CV_Object", 1000); 
  control_pub = n.advertise<planner::setPoints>("planner/setPoints", 1000);

  ros::Rate loop_rate(10);
  int thing = 0;
  while ( ros::ok() ) {
    ps3Control();
    setVisionObj("FW-Gate");
 
    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
