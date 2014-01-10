#include "Interface.h"

double depth;
double pressure;
double orientX;
double orientY;
double orientZ;
double orientW;

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

int main(int argc, char **argv) {
  ros::init(argc, argv, "Planner");
  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth_data", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure_data", 1000, setPressure);
  ros::Subscriber IMU_sub = n.subscribe("imu_data", 1000, setOrientation);

  ros::Publisher velocity_pub = n.advertise<simulator::ThrusterForces>("gazebo/thruster_forces", 1000);
  ros::Publisher CV_objs_pub = n.advertise<std_msgs::String>("planner/CV_Object", 1000); 

  ros::Rate loop_rate(10);
  int thing = 0;
  while ( ros::ok() ) {
    simulator::ThrusterForces msgAI;
    if (thing < 20) {
      msgAI.ty1 = 1.0;
      msgAI.ty2 = 1.0;
      thing = thing + 1;
    }
    else {
      msgAI.ty1 = 0.0;
      msgAI.ty2 = 0.0;
    }
    msgAI.tx1 = 0.0;
    msgAI.tz1 = 0.0;
    msgAI.tx2 = 0.0;
    msgAI.tz2 = 0.0;
    
    std_msgs::String msgCV;
    msgCV.data = "FW-Gate";
 
    velocity_pub.publish(msgAI);
    CV_objs_pub.publish(msgCV);

    ros::spinOnce();
    loop_rate.sleep();
  }
  ros::spin();
  return 0;
}
