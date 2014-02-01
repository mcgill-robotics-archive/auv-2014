#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <arduino_msgs/motor.h>
#include <arduino_msgs/solenoid.h>

ros::NodeHandle nh;
std_msgs::Int16 depth_msg;

Servo myservo[6];


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

void solenoidCb( const arduino_msgs::solenoid& msg){
digitalWrite(0,msg.torpedo0.data);
digitalWrite(1,msg.torpedo1.data);
digitalWrite(2,msg.dropper0.data);
digitalWrite(3,msg.dropper1.data);
digitalWrite(4,msg.grabber0.data);
digitalWrite(5,msg.grabber1.data);
}

ros::Subscriber<arduino_msgs::solenoid> solenoid_sub("/elec_interface/solenoic", &solenoidCb );
ros::Subscriber<arduino_msgs::motor> motor_sub("/elec_interface/motor", &motorCb );

void setup(){
 
  //MotorControl setup
  myservo[0].attach(2); 
  myservo[1].attach(3);  
  myservo[2].attach(4);  
  myservo[3].attach(5);  
  myservo[4].attach(6);  
  myservo[5].attach(7);
  
  //SolenoidValve setup
  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  
  //ros node initializtion
  nh.initNode();
  nh.subscribe(motor_sub);    //motor 
  nh.subscribe(solenoid_sub); // solenoid
}




void loop(){
  nh.spinOnce();
  delay(1);
}
