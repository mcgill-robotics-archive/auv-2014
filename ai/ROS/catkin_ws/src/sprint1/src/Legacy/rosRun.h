int main(int argc, char **argv)
{
  ros::init(argc, argv, "ROS");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("depth", 1000, setDepth);

  ros::spin();

  return 0;
}
