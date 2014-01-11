#include "Interface.h"
#include "planner/setPoints.h"
double depth;
double pressure;
double orientX;
double orientY;
double orientZ;
double orientW;


double XPos;
double YPos;
double ZPos;
double Yaw;
double Pitch;
double XSpeed;
double YSpeed;
double YawSpeed;

int XPosControl;
int YPosControl;
int ZPosControl;
int YawControl;
int PitchControl;
int XSpeedControl;
int YSpeedControl;
int YawSpeedControl;

Interface::Interface()
{
}

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

void setPoints(int pointControl[]) {
  XPosControl = pointControl[0];
  XPos = pointControl[1];

  YPosControl = pointControl[2];
  YPos = pointControl[3];

  ZPosControl = pointControl[4];
  ZPos = pointControl[5];

  YawControl = pointControl[6];
  Yaw = pointControl[7];

  PitchControl = pointControl[8];
  Pitch = pointControl[9];

  XSpeedControl = pointControl[10];
  XSpeed = pointControl[11];

  YSpeedControl = pointControl[12];
  YSpeed = pointControl[13];

  YawSpeedControl = pointControl[14];
  YawSpeed = pointControl[15];
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Planner");
  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth_data", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure_data", 1000, setPressure);
  ros::Subscriber IMU_sub = n.subscribe("imu_data", 1000, setOrientation);

  ros::Publisher ps3_pub = n.advertise<simulator::ThrusterForces>("gazebo/thruster_forces", 1000);
  ros::Publisher CV_objs_pub = n.advertise<std_msgs::String>("planner/CV_Object", 1000); 
  ros::Publisher control_pub = n.advertise<planner::setPoints>("planner/setPoints", 1000);

  ros::Rate loop_rate(10);
  int thing = 0;
  while ( ros::ok() ) {
    simulator::ThrusterForces msgPS3;
    if (thing < 20) {
      msgPS3.ty1 = 1.0;
      msgPS3.ty2 = 1.0;
      thing = thing + 1;
    }
    else {
      msgPS3.ty1 = 0.0;
      msgPS3.ty2 = 0.0;
    }
    msgPS3.tx1 = 0.0;
    msgPS3.tz1 = 0.0;
    msgPS3.tx2 = 0.0;
    msgPS3.tz2 = 0.0;

    planner::setPoints msgControl;
    msgControl.XPos.isActive = XPosControl;
    msgControl.XPos.data = XPos;
    msgControl.YPos.isActive = YPosControl;
    msgControl.YPos.data = YPos;
    msgControl.ZPos.isActive = ZPosControl;
    msgControl.ZPos.data = ZPos;
    msgControl.Yaw.isActive = YawControl;
    msgControl.Yaw.data = Yaw;
    msgControl.Pitch.isActive = PitchControl;
    msgControl.Pitch.data = Pitch;
    msgControl.XSpeed.isActive = XSpeedControl;
    msgControl.XSpeed.data = XSpeed;
    msgControl.YSpeed.isActive = YSpeedControl;
    msgControl.YSpeed.data = YSpeed;
    msgControl.YawSpeed.isActive = YawSpeedControl;
    msgControl.YawSpeed.data = YawSpeed;
    
    std_msgs::String msgCV;
    msgCV.data = "FW-Gate";
 
    ps3_pub.publish(msgPS3);
    CV_objs_pub.publish(msgCV);
    control_pub.publish(msgControl);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
