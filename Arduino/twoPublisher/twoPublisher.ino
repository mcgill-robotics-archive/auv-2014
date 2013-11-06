#include <ros.h>
#include <std_msgs/Int16.h>


ros::NodeHandle nh;
std_msgs::Int16 analog_reading1;
ros::Publisher publish_analog1("pin1",&analog_reading1);
std_msgs::Int16 analog_reading2;
ros::Publisher publish_analog2("pin2",&analog_reading2);
void setup(){
  nh.initNode();
  nh.advertise(publish_analog1);
  nh.advertise(publish_analog2);
}


void loop(){
  analog_reading1.data=analogRead(0);
  publish_analog1.publish(&analog_reading1);  
    analog_reading2.data=analogRead(1);
  publish_analog2.publish(&analog_reading2);  
  nh.spinOnce();
  delay(1000);
}
