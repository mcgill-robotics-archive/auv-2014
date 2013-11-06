#include <ros.h>
#include <std_msgs/Int16.h>

ros::NodeHandle nh;
std_msgs::Int16 value_msg;
ros::Publisher report("report", &value_msg);

void valueCb( const std_msgs::Int16& value ){
  value_msg.data= value.data;
}

ros::Subscriber<std_msgs::Int16> sub("value", &valueCb);

void setup(){
  DDRB=B00111111;
  nh.initNode();
  nh.subscribe(sub);
  nh.advertise(report);
}

void loop(){
  value_msg.data++;
  PORTB=(value_msg.data ^ B00111111);  
  report.publish(&value_msg);
  nh.spinOnce();
  delay(600);
}

