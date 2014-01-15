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


void setPoints(double pointControl[]) {
  planner::setPoints msgControl;

  msgControl.XPos.isActive = pointControl[0];
  msgControl.XPos.data = pointControl[1];;
  
  msgControl.YPos.isActive = pointControl[2];
  msgControl.YPos.data = pointControl[3];
  
  msgControl.ZPos.isActive = pointControl[4];
  msgControl.ZPos.data = pointControl[5];
  
  msgControl.Yaw.isActive = pointControl[6];
  msgControl.Yaw.data = pointControl[7];

  msgControl.Pitch.isActive = pointControl[8];
  msgControl.Pitch.data = pointControl[9];
  
  msgControl.XSpeed.isActive = pointControl[10];
  msgControl.XSpeed.data = pointControl[11];

  msgControl.YSpeed.isActive = pointControl[12];
  msgControl.YSpeed.data = pointControl[13];

  msgControl.YawSpeed.isActive = pointControl[14];
  msgControl.YawSpeed.data = pointControl[15];

  msgControl.Depth.isActive = pointControl[16];
  msgControl.Depth.data = pointControl[17];

  control_pub.publish(msgControl);
}

void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth){
	
	double pointControl[18] = {0, 0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,0 ,1 ,x_speed ,1,y_speed ,1, yaw_speed, 1,depth};
	setPoints(pointControl);
}


void setPosition(double x_pos, double y_pos, double z_pos, double pitch_angle, double yaw_angle){
	double pointControl[18] = {1, x_pos, 1, y_pos, 1, z_pos, 1, pitch_angle, 1, yaw_angle, 0, 0, 0, 0, 0, 0, 0, 0};
	setPoints(pointControl);
}



void ps3Control() {
  simulator::ThrusterForces msgPS3;
    msgPS3.ty1 = 40.0;
    msgPS3.ty2 = 40.0;
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

  std::cout<<"Starting Loader"<< std::endl; 
  Loader* loader = new Loader();
  Invoker* invoker = loader->getInvoker();
  invoker->StartRun();
  std::cout<<"Done Loader"<< std::endl;  

  ros::Rate loop_rate(10);
  int thing = 0;
  //while ( ros::ok() ) {
    //ps3Control();
    //setVisionObj("FW-Gate");
    //setVelocity(1, 1, 1, 1);
    //ros::spinOnce();
    //loop_rate.sleep();
  //}
  ros::spin();
  return 0;
}
