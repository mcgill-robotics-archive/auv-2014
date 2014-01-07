#include "Interface.h"
#include "std_msgs/String.h"

double Depth;
double Pressure;
double GlobalX;
double GlobalY;

Interface::Interface()
{
}

double getDepth()
{
    //ROS_INFO("I heard: [%f]", Depth);
    return Depth;
}

void setDepth(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
  Depth = msg->data;
  getDepth();
}

void setPosition(const geometry_msgs::Twist msg)
{
}

double GetX()
{
    return GlobalX;
}

double GetY()
{
    return GlobalY;
}
/*
Vector Velocity()
{
}
*/
double getPressure()
{
    //ROS_INFO("I heard: [%f]", Pressure);
    return Depth;
}


void setPressure(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
  Pressure = msg->data;
}

void TODO(const geometry_msgs::Twist msg){}


double orientX; double orientY; double orientZ; double orientW;
void setOrientation(const geometry_msgs::Quaternion msg) {
  orientX = msg.x;
  orientY = msg.y;
  orientZ = msg.z;
  orientW = msg.w;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AI_Interface");

  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth_data", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure_data", 1000, setPressure);
  ros::Subscriber IMU_sub = n.subscribe("imu_data", 1000, setOrientation);
  ros::Subscriber CV_sub = n.subscribe("CV", 1000, TODO);

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("AI", 1000);
  ros::Publisher CV_objs_pub = n.advertise<std_msgs::Int64>("CV_Objs", 1000); 

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    geometry_msgs::Twist msgAI;
    msgAI.linear.x = 1.0;
    msgAI.linear.y = 1.0;
    msgAI.linear.z = 1.0;
    msgAI.angular.x = 1.0;
    msgAI.angular.y = 1.0;
    msgAI.angular.z = 1.0;

    std_msgs::Int64 msgCV;
    msgCV.data = 12;
 
    velocity_pub.publish(msgAI);
    CV_objs_pub.publish(msgCV);

    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
