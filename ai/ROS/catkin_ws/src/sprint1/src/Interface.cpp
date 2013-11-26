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

void setDepth(const sprint1::Depth::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->data);
  Depth = msg->data;
  getDepth();
}

void setPosition(const geometry_msgs::Twist msg)
{
   GlobalX = msg.linear.x;
   GlobalY = msg.linear.y;
   ROS_INFO("%f -- %f", msg.linear.x, msg.linear.y);
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


void setPressure(const sprint1::Pressure::ConstPtr& msg)
{
  //ROS_INFO("I heard: [%f]", msg->data);
  Pressure = msg->data;
  getPressure();
}

void TODO(const geometry_msgs::Twist msg){}

void setOrientation(const geometry_msgs::Pose msg){}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AI_Interface");

  ros::NodeHandle n;

  ros::Subscriber depth_sub = n.subscribe("depth", 1000, setDepth);
  ros::Subscriber pressure_sub = n.subscribe("pressure", 1000, setPressure);
  ros::Subscriber CV_sub = n.subscribe("CV", 1000, TODO);
  ros::Subscriber DVL_sub = n.subscribe("DVL", 1000, setPosition);
  ros::Subscriber IMU_sub = n.subscribe("IMU", 1000, setOrientation);

  ros::Publisher velocity_pub = n.advertise<geometry_msgs::Twist>("AI", 1000);
  ros::Publisher CV_objs_pub = n.advertise<sprint1::CVQuery>("CV_Objs", 1000); 

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

    sprint1::CVQuery msgCV;
    msgCV.queries.push_back(1);
    msgCV.queries.push_back(2);
    ROS_INFO("%i is the object we are looking for", msgCV.queries[0]);

    velocity_pub.publish(msgAI);
    CV_objs_pub.publish(msgCV);

    ros::spinOnce();

    loop_rate.sleep();
  }

  ros::spin();

  return 0;
}
