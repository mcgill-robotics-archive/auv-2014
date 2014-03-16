#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <controls/motorCommands.h>
#include <arduino_msgs/solenoid.h>

ros::NodeHandle nh;
std_msgs::Int16 depth_msg;
std_msgs::Int16 batteryCurrent;
std_msgs::Int16 batteryVoltage0;
std_msgs::Int16 batteryVoltage1;
//PINOUT:
//Motor control   : 2 - 7
//Solenoid valve  : 14 - 19 
//Depth sensor    :  A0
//Battery voltage : A3, A4
//Battery current : A5
Servo myservo[6];
const static int depthPin = A0;      // select the input pin for the potentiometer
const static int motorControlPin = 2 ; 
const static int solenoidValvePin = 14;
const static int battPin = A3;

unsigned long depthSensorSchedule;
unsigned long batteryVoltageSchedule;
unsigned long batteryCurrentSchedule;
boolean killSwitchEngaged = false;

// Moving Average filter for depth data
const static int WINDOW_WIDTH = 3;
int depthReadings[WINDOW_WIDTH] = {0, 0, 0, 0, 0};
int depthIndex = 0;
int depthAverage = 0;

int boundCheck(int x){
  if(x> 500 || x< -500){
    char msg[70];
    String("Motor Speed out of bound: " + String(x) +" !").toCharArray(msg,70);
    nh.logerror(msg);
    return 0;  
  }
  return x;
}

void motorCb( const controls::motorCommands& msg){
  myservo[0].writeMicroseconds(1500 + boundCheck(msg.cmd_x1));
  myservo[1].writeMicroseconds(1500 + boundCheck(msg.cmd_x2));
  myservo[2].writeMicroseconds(1500 + boundCheck(msg.cmd_y1));
  myservo[3].writeMicroseconds(1500 + boundCheck(msg.cmd_y2));
  myservo[4].writeMicroseconds(1500 + boundCheck(msg.cmd_z1));
  myservo[5].writeMicroseconds(1500 + boundCheck(msg.cmd_z2));
}

void solenoidCb( const arduino_msgs::solenoid& msg){
  digitalWrite(14,msg.torpedo0.data);
  digitalWrite(15,msg.torpedo1.data);
  digitalWrite(16,msg.grabber0.data);
  digitalWrite(17,msg.grabber1.data);
  digitalWrite(18,msg.dropper0.data);
  digitalWrite(19,msg.dropper1.data);
}

void killSwitchCb( const std_msgs::Empty& msg){
  if(!killSwitchEngaged){
    nh.logwarn("Kill switch engaged! Initiating self-destruction!");
    killSwitchEngaged = true;
    digitalWrite(0,HIGH);
  }
}

ros::Publisher depth("/arduino/depth", &depth_msg);  // Publish the depth topic
ros::Publisher battVoltPub0("/arduino/batteryVoltage0", &batteryVoltage0);
ros::Publisher battVoltPub1("/arduino/batteryVoltage1", &batteryVoltage1);
ros::Publisher battCurrPub("/arduino/batteryCurrent", &batteryCurrent);
ros::Subscriber<arduino_msgs::solenoid> solenoid_sub("/arduino/solenoid", &solenoidCb );
ros::Subscriber<controls::motorCommands> motor_sub("/arduino/motor", &motorCb );
ros::Subscriber<std_msgs::Empty> killSwitch_sub("/arduino/KillSwitch", &killSwitchCb);

void setup(){
  for(int i = 0; i<6; i++){
    //MotorControl setup
    myservo[i].attach(i+2); 
    //SolenoidValve setup  
    pinMode(14+i,OUTPUT);
   }
  //ros node initializtion
  nh.initNode();
  nh.advertise(depth);        //depth sensor
  
  nh.advertise(battVoltPub0);     //battery level
  nh.advertise(battVoltPub1);
  nh.advertise(battCurrPub);
  
  nh.subscribe(killSwitch_sub);// kill switch
  nh.subscribe(motor_sub);    //motor 
  nh.subscribe(solenoid_sub); // solenoid
}

void loop(){
  long currentTime = millis();
  
  if(batteryCurrentSchedule < currentTime){
    batteryCurrent.data = analogRead(battPin + 2);
    battCurrPub.publish(&batteryCurrent);    
    batteryCurrentSchedule += 100;     //Update at 10 Hz
  }
  
  if(depthSensorSchedule < currentTime){
    int depthReading = analogRead(depthPin );
    depth_msg.data = depthAverage + (depthReading - depthReadings[depthIndex]) / WINDOW_WIDTH;
    depth.publish(&depth_msg);

    // Update list of depth readings
    depthReadings[depthIndex] = depthReading;
    depthIndex = (depthIndex + 1) % WINDOW_WIDTH;

    depthSensorSchedule += 200;        //Update at 5Hz  
  }

  if(batteryVoltageSchedule < currentTime){
    batteryVoltage0.data = analogRead(battPin + 0);
    batteryVoltage1.data = analogRead(battPin + 1);
    
    battVoltPub0.publish(&batteryVoltage0);
    battVoltPub1.publish(&batteryVoltage1);
    
    batteryVoltageSchedule += 1000;     //Update at 1 Hz
  }  
  nh.spinOnce();
  delay(1);
}
