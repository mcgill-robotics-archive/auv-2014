/*
 *  Proximity IR sensor is used to detect when someone cross a door.
 *  The light changes state when someone cross the door
 *
 * Light pin 13
 * IR sensor pin A0
 * 
 * HOW
 * $ roscore
 * $ rosrun rosserial_python serial_node.py /dev/ttyUSB0
 *
 * DISPLAY IR sensor value
 * $ ros topic echo prox_val
 *
 * CHANGE light state
 * $ rostopic pub_switch toggle_light std_msgs/Empty --once
 *
 * Felix Dube 
 * November 3, 2013
 *
 */

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>

ros::NodeHandle  nh;

//Publisher that send an empty message to toggle_light when the proximity sensor detects someone
std_msgs::Empty prox_msg;
ros::Publisher pub_switch("toggle_light", &prox_msg);

//Subscriber to toggle_light and change the state of the light when it receive an empty message
ros::Subscriber<std_msgs::Empty> sub_light("toggle_light", messageCb );

//Publisher of the proximity sensor value to prox_val
std_msgs::Float32 proxVal_msg;
ros::Publisher prox("prox_val", &proxVal_msg);

void messageCb( const std_msgs::Empty& toggle_msg){
  digitalWrite(13, HIGH-digitalRead(13));   // changes the state of the light
}




void setup()
{
  pinMode(A0, INPUT);  //IR sensor
  pinMode(13, OUTPUT);  //Light
  
  nh.initNode();
  nh.advertise(pub_switch);
  nh.advertise(prox);
  nh.subscribe(sub_light);
}

void loop()
{
  
  if(analogRead(A0)>200){
    pub_switch.publish( &prox_msg );
    delay(500);  //Makes sure the light does not change more than one when someone is detected by the IR sensor
  }
  
  proxVal_msg.data = analogRead(A0);
  prox.publish( &proxVal_msg);
  
  nh.spinOnce();
  delay(100);
}
