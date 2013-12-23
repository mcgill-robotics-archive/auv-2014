#include <ros.h>
#include <std_msgs/Int16.h>
int val;
ros::NodeHandle nh;
std_msgs::Int16 value_msg;
ros::Publisher report("IR/data", &value_msg);

void valueCb( const std_msgs::Int16& value ){
  val=value.data;
  PORTA=seg(val);
}

ros::Subscriber<std_msgs::Int16> sub("led/val", &valueCb);
int seg(int index){
switch (index) {
    case 0:
      return B00001010;
    case 1:
      return B11101011;
    case 2:
      return B01000110;
    case 3:
      return B01100010;
    case 4:
      return B10100011;
    case 5:
      return B00110010;
    case 6:
      return B00010010;
    case 7:
      return B01101011;
    case 8:
      return B00000010;
    case 9:
      return B00100010;
    case 10:
      return B00000011;
    case 11:
      return B10010010;
    case 12:
      return B00011110;
    case 13:
      return B11000010;
    case 14:
      return B00010110;
    case 15:
      return B00010111;  
    default: 
      return B11111111;
  }
}

void setup(){
  DDRA=B11111111;
  PORTA=B11111111;
  nh.initNode();
  nh.advertise(report);
  nh.subscribe(sub);
}

void loop(){
  int value=map(analogRead(1),0,600,0,2000);
  value_msg.data=value;
  report.publish(&value_msg);
  nh.spinOnce();
  delay(100);
}

