#include <ros.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
int state = 1;
int value = 0;

ros::NodeHandle nh;

void toggle( const std_msgs::Empty& toggle_msg){
  state^=1;
}
ros::Subscriber<std_msgs::Empty> sub1("toggle", &toggle);

void setValue(const std_msgs::Int16& value_msg ){
   value=value_msg.data;
}
ros::Subscriber<std_msgs::Int16> sub2("value", &setValue);

void setup()
{
  DDRD=B00111111;
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
}

void loop()
{
  if(state){
  PORTB=(value & B00111111);  
  }
  nh.spinOnce();
  delay(1);
}

