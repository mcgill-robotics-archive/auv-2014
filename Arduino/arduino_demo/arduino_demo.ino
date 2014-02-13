#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <controls/motorCommands.h>
#include <arduino_msgs/solenoid.h>
#include <std_msgs/Float32.h>

#define MAX_ANALOG 1023  // in bits
#define MAX_ARDUINO_VOLTAGE 5000 // in mV
#define RESISTANCE 100.0 // In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.

#define BASE_CURRENT 4.0
#define CURRENT_RANGE 16.0   // from 4.0 mA to 20.0 mA
#define MAX_DEPTH 914.4      // In centimeters. 30 feet
#define OFFSET 7.0          // Just substract it. May need to be recalculated when circuit is built.
#define BATT_VOLT_CONVERSION 3.14 //mock constant change later


ros::NodeHandle nh;
std_msgs::Int16 depth_msg;
std_msgs::Float32 batteryLevel0;
std_msgs::Float32 batteryLevel1;

Servo myservo[6];
int sensorPin = A0;      // select the input pin for the potentiometer
float sensorValue = 0;  // variable to store the value coming from the sensor
long depthSensorSchedule;
long batteryLevelSchedule;


int boundCheck(int x){
  if(x> 500 || x< -500){
    nh.logerror("Motor Speed out of bound!");
    return 0;  
  }
  return x;
}

void motorCb( const controls::motorCommands& msg){
  myservo[0].writeMicroseconds(1500 + boundCheck(msg.cmd_x1));
  myservo[1].writeMicroseconds(1500 + boundCheck(msg.cmd_x2));  // blink the led
  myservo[2].writeMicroseconds(1500 + boundCheck(msg.cmd_y1));
  myservo[3].writeMicroseconds(1500 + boundCheck(msg.cmd_y2));
  myservo[4].writeMicroseconds(1500 + boundCheck(msg.cmd_z1));
  myservo[5].writeMicroseconds(1500 + boundCheck(msg.cmd_z2));
}

void solenoidCb( const arduino_msgs::solenoid& msg){
digitalWrite(14,msg.torpedo0.data);
digitalWrite(15,msg.torpedo1.data);
digitalWrite(16,msg.dropper0.data);
digitalWrite(17,msg.dropper1.data);
digitalWrite(18,msg.grabber0.data);
digitalWrite(19,msg.grabber1.data);
}

void killSwitchCb(const std_msgs::Bool& msg){
digitalWrite(0,msg.data);
}

ros::Publisher depth("/arduino/depth", &depth_msg);  // Publish the depth topic
ros::Publisher battPub0("/arduino/batteryLevel0", &batteryLevel0);
ros::Publisher battPub1("/arduino/batteryLevel1", &batteryLevel1);
ros::Subscriber<arduino_msgs::solenoid> solenoid_sub("/arduino/solenoid", &solenoidCb );
ros::Subscriber<controls::motorCommands> motor_sub("/arduino/motor", &motorCb );
ros::Subscriber<std_msgs::bool> killSwitch_sub("/arduino/KillSwitch", &motorCb );

void setup(){
 
  //MotorControl setup
  myservo[0].attach(0); 
  myservo[1].attach(1);  
  myservo[2].attach(2);  
  myservo[3].attach(3);  
  myservo[4].attach(4);  
  myservo[5].attach(5);
  
  //SolenoidValve setup
  pinMode(14,OUTPUT);
  pinMode(15,OUTPUT);
  pinMode(16,OUTPUT);
  pinMode(17,OUTPUT);
  pinMode(18,OUTPUT);
  pinMode(19,OUTPUT);
  
  //ros node initializtion
  nh.initNode();
  nh.advertise(depth);        //depth sensor
  
  nh.advertise(battPub0);     //battery level
  nh.advertise(battPub1);
  
  nh.subscribe(motor_sub);    //motor 
  nh.subscribe(solenoid_sub); // solenoid
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
    
    depthSensorSchedule += 250;        //Update at 10Hz  
  }
  
  if(batteryLevelSchedule < currentTime){
    batteryLevel0.data = analogRead(A3) * BATT_VOLT_CONVERSION;
    batteryLevel1.data = analogRead(A4) * BATT_VOLT_CONVERSION;
    
    battPub0.publish(&batteryLevel0);
    battPub1.publish(&batteryLevel1);
    
    batteryLevelSchedule += 10000;     //Update at 0.1 Hz
  }  
  nh.spinOnce();
  delay(1);
}
