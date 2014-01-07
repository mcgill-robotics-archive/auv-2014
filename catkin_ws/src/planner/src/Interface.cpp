#include "Interface.h"
#include "std_msgs/String.h"

double Depth;
double Pressure;

double XPos;
double YPos;

double XOrient;
double YOrient;
double ZOrient;
double WOrient;

Interface::Interface()
{
}

double getDepth() {
    return Depth;
}

void setDepth(const std_msgs::Float64::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->data);
  Depth = msg->data;
}

void setPosition(const geometry_msgs::Twist msg) {
}

double GetX() {
    return XPos;
}

double GetY() {
    return YPos;
}

double getPressure() {
    return Pressure;
}

void setPressure(const std_msgs::Float64::ConstPtr& msg) {
  ROS_INFO("I heard: [%f]", msg->data);
  Pressure = msg->data;
}

void TODO(const geometry_msgs::Twist msg){}

void setOrientation(const geometry_msgs::Quaternion msg) {
  XOrient = msg.x;
  YOrient = msg.y;
  ZOrient = msg.z;
  WOrient = msg.w;
}

double XVal = 1.0;
double YVal = 2.0;
double ZVal = 3.0;
double YawVal = 4.0;
double PitchVal = 5.0;
double XSpeed = 6.0;
double YSpeed = 7.0;
double ZSpeed = 8.0;
double YawSpeed = 9.0;
double PitchSpeed = 0.0;
bool XValControl = 1;
bool YValControl = 0;
bool ZValControl = 1;
bool YawValControl = 0;
bool PitchValControl = 1;
bool XSpeedControl = 1;
bool YSpeedControl = 0;
bool ZSpeedControl = 0;
bool YawSpeedControl = 0;
bool PitchSpeedControl = 1;
void setMotorPass(int values[], bool control[]) {
  XVal       = values[0];
  YVal       = values[1];
  ZVal       = values[2];
  YawVal     = values[3];
  PitchVal   = values[4];
  XSpeed     = values[5];
  YSpeed     = values[6];
  ZSpeed     = values[7];
  YawSpeed   = values[8];
  PitchSpeed = values[9];

  XValControl       = control[0];
  YValControl       = control[1];
  ZValControl       = control[2];
  YawValControl     = control[3];
  PitchValControl   = control[4];
  XSpeedControl     = control[5];
  YSpeedControl     = control[6];
  ZSpeedControl     = control[7];
  YawSpeedControl   = control[8];
  PitchSpeedControl = control[9];
}

std::string CVObj = "thingy";
void setVisionObject(std::string obj) {
  CVObj = obj;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "AI_Interface");
  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth_data", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure_data", 1000, setPressure);
  ros::Subscriber IMU_sub = n.subscribe("imu_data", 1000, setOrientation);
  ros::Subscriber CV_sub = n.subscribe("CV", 1000, TODO);

  ros::Publisher CV_objs_pub = n.advertise<std_msgs::String>("CV_Objs", 1000);
  ros::Publisher Motor_Control_pub = n.advertise<planner::MotorControl>("Control_Commands", 1000);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    std_msgs::String msgCV;
      msgCV.data = CVObj;
    CV_objs_pub.publish(msgCV);

    planner::MotorControl msgMotor;
      msgMotor.XPos.isActive = XValControl;
      msgMotor.XPos.data = XVal;
      msgMotor.YPos.isActive = YValControl;
      msgMotor.YPos.data = YVal;
      msgMotor.ZPos.isActive = ZValControl;
      msgMotor.ZPos.data = ZVal;
      msgMotor.Yaw.isActive = YawValControl;
      msgMotor.Yaw.data = YawVal;
      msgMotor.Pitch.isActive = PitchValControl;
      msgMotor.Pitch.data = PitchVal;
      msgMotor.XSpeed.isActive = XSpeedControl;
      msgMotor.XSpeed.data = XSpeed;
      msgMotor.YSpeed.isActive = YSpeedControl;
      msgMotor.YSpeed.data = YSpeed;
      msgMotor.ZSpeed.isActive = ZSpeedControl;
      msgMotor.ZSpeed.data = ZSpeed;
      msgMotor.YawSpeed.isActive = YawSpeedControl;
      msgMotor.YawSpeed.data = YawSpeed;
      msgMotor.PitchSpeed.isActive = PitchSpeedControl;
      msgMotor.PitchSpeed.data = PitchSpeed;
    Motor_Control_pub.publish(msgMotor);

    ros::spinOnce();
    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
