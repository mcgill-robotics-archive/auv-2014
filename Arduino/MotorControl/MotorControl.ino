#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <arduino_msgs/motor.h>
#include <arduino_msgs/solenoid.h>
Servo myservo[6];
ros::NodeHandle nh;

int boundCheck(int x){
if(x> 500 || x< -500){
nh.logerror("Motor speed out of bound ! " + x);
return 0;
}
return x;
}



void motorCb( const arduino_msgs::motor& msg){
  myservo[0].writeMicroseconds(1500 + msg.x_p);
  myservo[1].writeMicroseconds(1500 + msg.x_n);  // blink the led
  myservo[2].writeMicroseconds(1500 + msg.y_p);
  myservo[3].writeMicroseconds(1500 + msg.y_n);
  myservo[4].writeMicroseconds(1500 + msg.z_p);
  myservo[5].writeMicroseconds(1500 + msg.z_n);
}

ros::Subscriber<arduino_msgs::motor> sub("/elec_interface/motor", &motorCb );

void setup(){
  myservo[0].attach(2); 
  myservo[1].attach(3);  
  myservo[2].attach(4);  
  myservo[3].attach(5);  
  myservo[4].attach(6);  
  myservo[5].attach(7);
  nh.initNode();
  nh.subscribe(sub);
}
void loop(){
nh.spinOnce();
delay(1);
}
