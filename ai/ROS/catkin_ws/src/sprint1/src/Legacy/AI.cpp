
void depthCallback(const std_msgs::Float64::ConstPtr& msg)
{
  ROS_INFO("I heard: [%f]", msg->data);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "AI");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("depth", 1000, depthCallback);

  ros::spin();

  return 0;
}
