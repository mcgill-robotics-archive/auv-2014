#include <ros.h>
#include <std_msgs/Int16.h>
#include <Servo.h>
Servo motorarray[6];
Servo grub[3];
std_msgs::Int16 batteryState;
ros::NodeHandle nh;
ros::Publisher reportBattery("Battery",&batteryState);

void motorCb(const std_msgs::Int16& motorstate_msg){
    short msg= motorstate_msg.data;
    char motorNum= msg & B0111;
    msg = msg >> 4;
    motorarray[motorNum].writeMicroseconds(msg);
}

ros::Subscriber<std_msgs::Int16> motor("motor",&motorCb);


void loop(){
  if((millis()%10000 ==0)){
  //read battery state
  }
  nh.spinOnce();
  delay(1);
}

void setup(){
  nh.initNode();
  nh.advertise(reportBattery);
  nh.subscribe(motor);
}
