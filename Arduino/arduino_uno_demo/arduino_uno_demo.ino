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
std_msgs::Int16 batteryCurrent_msg;
std_msgs::Int16 batteryVoltage0_msg;
std_msgs::Int16 batteryVoltage1_msg;
std_msgs::Int16 temperature_msg;
std_msgs::Int32 pressure_msg;

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
//unsigned long batteryCurrentSchedule;
unsigned long temperaturePressureSechedule;
boolean killSwitchEngaged = false;
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
  digitalWrite(14,msg.torpedo1.data);
  digitalWrite(15,msg.torpedo2.data);
  digitalWrite(16,msg.grabber1.data);
  digitalWrite(17,msg.grabber2.data);
  digitalWrite(18,msg.dropper1.data);
  digitalWrite(19,msg.dropper2.data);
}


ros::Publisher depth("/depth", &depth_msg);  // Publish the depth topic
ros::Publisher battVoltPub0("/batteryVoltage0", &batteryVoltage0_msg);
ros::Publisher battVoltPub1("/batteryVoltage1", &batteryVoltage1_msg);
ros::Publisher temperaturePub("/temperature", &temperature_msg);
ros::Publisher pressurePub("/pressure", &pressure_msg);
ros::Subscriber<arduino_msgs::solenoid> solenoid_sub("/solenoid", &solenoidCb );
ros::Subscriber<controls::motorCommands> motor_sub("/motor", &motorCb );
void setup(){
  for(int i = 0; i<6; i++){
    //MotorControl setup
    myservo[i].attach(i+2); 
    //SolenoidValve setup  
    pinMode(14+i,OUTPUT);
   }
  //ros node initializtion
  bmp085Calibration();
  nh.initNode();
  nh.advertise(depth);        //depth sensor
  
  nh.advertise(battVoltPub0);     //battery level
  nh.advertise(battVoltPub1);
  nh.advertise(temperaturePub);
  nh.advertise(pressurePub);
  nh.subscribe(motor_sub);    //motor 
  nh.subscribe(solenoid_sub); // solenoid
}

void loop(){
  long currentTime = millis();
 
  if(temperaturePressureSechedule < currentTime){
    
   pressure_msg.data= bmp085GetPressure(bmp085ReadUP());
   temperature_msg.data = bmp085GetTemperature(bmp085ReadUT());
   
   temperaturePub.publish(&temperature_msg);
   pressurePub.publish(&pressure_msg);
   
   temperaturePressureSechedule += 1000;
  }
  
  /*
  if(batteryCurrentSchedule < currentTime){
    batteryCurrent.data = analogRead(battPin + 2);
    battCurrPub.publish(&batteryCurrent);    
    batteryCurrentSchedule += 100;     //Update at 10 Hz
  }
  */

  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(depthPin );
    depth.publish(&depth_msg);
    depthSensorSchedule += 200;        //Update at 5Hz  
  }

  if(batteryVoltageSchedule < currentTime){
    batteryVoltage0_msg.data = analogRead(battPin + 0);
    batteryVoltage1_msg.data = analogRead(battPin + 1);
    
    battVoltPub0.publish(&batteryVoltage0_msg);
    battVoltPub1.publish(&batteryVoltage1_msg);
    
    batteryVoltageSchedule += 1000;     //Update at 1 Hz
  }  
  nh.spinOnce();
}
