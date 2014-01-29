#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <arduino_msgs/motor.h>
#include <arduino_msgs/solenoid.h>
#include <arduino_msgs/batteryLevel.h>

Servo myservo[6];


#define MAX_ANALOG 1023  // in bits
#define MAX_ARDUINO_VOLTAGE 5000 // in mV
#define RESISTANCE 100.0 // In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.

#define BASE_CURRENT 4.0
#define CURRENT_RANGE 16.0   // from 4.0 mA to 20.0 mA
#define MAX_DEPTH 914.4      // In centimeters. 30 feet
#define OFFSET 7.0          // Just substract it. May need to be recalculated when circuit is built.
std_msgs::Int16 depth_msg;
ros::Publisher depth("depth", &depth_msg);  // Publish the depth topic

int sensorPin = A0;      // select the input pin for the potentiometer
float sensorValue = 0;  // variable to store the value coming from the sensor
long depthSensorSchedule = 0;
long batteryLevelSchedule = 0;
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
  myservo[0].attach(2); 
  myservo[1].attach(3);  
  myservo[2].attach(4);  
  myservo[3].attach(5);  
  myservo[4].attach(6);  
  myservo[5].attach(7);
  pinMode(0,OUTPUT);
  pinMode(1,OUTPUT);
  pinMode(2,OUTPUT);
  pinMode(3,OUTPUT);
  pinMode(4,OUTPUT);
  pinMode(5,OUTPUT);
  nh.initNode();
  nh.advertise(depth);
  nh.subscribe(motor_sub);
  nh.subscribe(solenoid_sub);
}
void loop(){
  long currentTime = millis();
  
  if(depthSensorSchedule < currentTime){
  sensorValue = analogRead(sensorPin);                                             // ADC bits
  sensorValue = sensorValue*MAX_ARDUINO_VOLTAGE/(float)MAX_ANALOG;                 // From ADC to voltage in mV
  sensorValue = sensorValue/RESISTANCE;                                            // Get current in mA
  sensorValue = (int)(((sensorValue-BASE_CURRENT)/CURRENT_RANGE)*MAX_DEPTH);       // C0nvert the current to depth
  
  depth_msg.data = sensorValue - OFFSET;
  depth.publish(&depth_msg);
  depthSensorSchedule += 100;  
  }
  
  if(batteryLevelSchedule < currentTime){
  
  batteryLevelSchedule += 10000;
  }  
  nh.spinOnce();
  delay(1);
}
