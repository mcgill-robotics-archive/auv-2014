#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <controls/motorCommands.h>
#include <robosub_msg/solenoid.h>

//Pin definitions 

  //PWM MOTOR
  #define MOTOR_PIN_SU_ST 4
  #define MOTOR_PIN_SU_PO 5
  #define MOTOR_PIN_SW_BO 2
  #define MOTOR_PIN_SW_ST 3
  #define MOTOR_PIN_HE_BO 0
  #define MOTOR_PIN_HE_ST 1

  //SOLENOID  
  #define SOLENOID_PIN_D_1 6 
  #define SOLENOID_PIN_D_2 7
  #define SOLENOID_PIN_G_1 8
  #define SOLENOID_PIN_G_2 9
  #define SOLENOID_PIN_T_1 10
  #define SOLENOID_PIN_T_2 11

  //ANALOG
  #define VOLTAGE_PIN_1 A0
  #define VOLTAGE_PIN_2 A1
  #define DEPTH_SENSOR_PIN A2
  #define GRABBER_SWTCH_PIN_1 A3
  #define GRABBER_SWTCH_PIN_2 A4
  #define TEMPERATURE_PIN_1 A5
  #define TEMPERATURE_PIN_2 A6
  #define TEMPERATURE_PIN_3 A7
  #define TEMPERATURE_PIN_4 A8
  #define TEMPERATURE_PIN_5 A9 
  
//TIME INTERVAL(unit microsecond)
  #define MOTOR_TIMEOUT 4000          //amount of no signal required to start to reset motors 
  #define TEMPERATURE_INTERVAL 1000   //amount of delay between each temeperatures read
  #define VOLTAGE_INTERVAL 1000       //amount of delay between each voltages read
  #define DEPTH_INTERVAL 200          //amount of delay between each depth read
  
const double TEMP_RATIO = 0.322265625;    
const double VOLT_RATIO = (3.3*30.9 * 24.12) / (3.9 * 1024.0 * 23.46); //(teensy voltage * totoal resistance / (single resisitance * max bit))
  
  
ros::NodeHandle nh;
std_msgs::Int16 depth_msg;
std_msgs::Float32 batteryVoltage1_msg;
std_msgs::Float32 batteryVoltage2_msg;
std_msgs::Float32 temperature1_msg;
std_msgs::Float32 temperature2_msg;
std_msgs::Float32 temperature3_msg;
std_msgs::Float32 temperature4_msg;
std_msgs::Float32 temperature5_msg;

Servo myservo[6];

unsigned long depthSensorSchedule = 0;
unsigned long batteryVoltageSchedule = 0;
unsigned long temperatureSechedule = 0;
unsigned long lastMotorCommand = 0;

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
  lastMotorCommand = millis();
  myservo[0].writeMicroseconds(1476 + boundCheck(msg.cmd_surge_starboard));
  myservo[1].writeMicroseconds(1483 + boundCheck(msg.cmd_surge_port));
  myservo[2].writeMicroseconds(1471 + boundCheck(msg.cmd_sway_bow));
  myservo[3].writeMicroseconds(1476 + boundCheck(msg.cmd_sway_stern));
  myservo[4].writeMicroseconds(1489 + boundCheck(msg.cmd_heave_bow));
  myservo[5].writeMicroseconds(1508 + boundCheck(msg.cmd_heave_stern));
}

void resetMotor(){
  myservo[0].writeMicroseconds(1476);
  myservo[1].writeMicroseconds(1483);
  myservo[2].writeMicroseconds(1471);
  myservo[3].writeMicroseconds(1476);
  myservo[4].writeMicroseconds(1489);
  myservo[5].writeMicroseconds(1508);
}



void solenoidCb( const robosub_msg::solenoid& msg){
  digitalWrite(SOLENOID_PIN_T_1,msg.torpedo1.data);
  digitalWrite(SOLENOID_PIN_T_2,msg.torpedo2.data);
  digitalWrite(SOLENOID_PIN_G_1,msg.grabber1.data);
  digitalWrite(SOLENOID_PIN_G_2,msg.grabber2.data);
  digitalWrite(SOLENOID_PIN_D_1,msg.dropper1.data);
  digitalWrite(SOLENOID_PIN_D_2,msg.dropper2.data);
}


ros::Publisher depthPub("/electrical_interface/depth", &depth_msg);  // Publish the depth topic

ros::Publisher voltagePub1("/electrical_interface/batteryVoltage1", &batteryVoltage1_msg);
ros::Publisher voltagePub2("/electrical_interface/batteryVoltage2", &batteryVoltage2_msg);

/*
ros::Publisher temperaturePub1("/electrical_interface/temperature1", &temperature1_msg);
ros::Publisher temperaturePub2("/electrical_interface/temperature2", &temperature2_msg);
ros::Publisher temperaturePub3("/electrical_interface/temperature3", &temperature3_msg);
ros::Publisher temperaturePub4("/electrical_interface/temperature4", &temperature4_msg);
ros::Publisher temperaturePub5("/electrical_interface/temperature5", &temperature5_msg);
*/

ros::Subscriber<robosub_msg::solenoid> solenoidSub("/electrical_interface/solenoid", &solenoidCb );
ros::Subscriber<controls::motorCommands> motorSub("/electrical_interface/motor", &motorCb );


void setup(){
  myservo[0].attach(MOTOR_PIN_SU_ST);
  myservo[1].attach(MOTOR_PIN_SU_PO);
  myservo[2].attach(MOTOR_PIN_SW_BO);
  myservo[3].attach(MOTOR_PIN_SW_ST);
  myservo[4].attach(MOTOR_PIN_HE_BO);
  myservo[5].attach(MOTOR_PIN_HE_ST);

  resetMotor();

  pinMode(SOLENOID_PIN_T_1,OUTPUT);
  pinMode(SOLENOID_PIN_T_2,OUTPUT);
  pinMode(SOLENOID_PIN_D_1,OUTPUT);
  pinMode(SOLENOID_PIN_D_2,OUTPUT);
  pinMode(SOLENOID_PIN_G_1,OUTPUT);
  pinMode(SOLENOID_PIN_G_2,OUTPUT);


    
  //ros node initialization
  nh.initNode();
  
  //ros publisher initialization
  nh.advertise(depthPub);        //depth sensor
  nh.advertise(voltagePub1);     //battery level
  nh.advertise(voltagePub2);
  /*
  nh.advertise(temperaturePub1);
  nh.advertise(temperaturePub2);
  nh.advertise(temperaturePub3);
  nh.advertise(temperaturePub4);
  nh.advertise(temperaturePub5);
  */
  
  //ros subscribe initialization
  nh.subscribe(motorSub);  
  nh.subscribe(solenoidSub);

  resetMotor();

}

void loop(){
  
  long currentTime = millis();
  /*
  //temperature sensing
  if(temperatureSechedule < currentTime){
   temperature1_msg.data = analogRead(TEMPERATURE_PIN_1) * TEMP_RATIO -50; 
   temperaturePub1.publish(&temperature1_msg);
   temperature2_msg.data = analogRead(TEMPERATURE_PIN_2) * TEMP_RATIO -50; 
   temperaturePub2.publish(&temperature2_msg);
   temperature3_msg.data = analogRead(TEMPERATURE_PIN_3) * TEMP_RATIO -50; 
   temperaturePub3.publish(&temperature3_msg);
   temperature4_msg.data = analogRead(TEMPERATURE_PIN_4) * TEMP_RATIO -50; 
   temperaturePub4.publish(&temperature4_msg);
   temperature5_msg.data = analogRead(TEMPERATURE_PIN_5) * TEMP_RATIO -50; 
   temperaturePub5.publish(&temperature5_msg);
   temperatureSechedule += TEMPERATURE_INTERVAL;
  }
  */
  
  //depth sensing  
  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(DEPTH_SENSOR_PIN);
    depthPub.publish(&depth_msg);
    depthSensorSchedule += DEPTH_INTERVAL;   
  }

  //voltages sensing
  if(batteryVoltageSchedule < currentTime){
    batteryVoltage1_msg.data = analogRead(VOLTAGE_PIN_1) * VOLT_RATIO;
    batteryVoltage2_msg.data = analogRead(VOLTAGE_PIN_2) * VOLT_RATIO;
    
    voltagePub1.publish(&batteryVoltage1_msg);
    voltagePub2.publish(&batteryVoltage2_msg);
    
    batteryVoltageSchedule += VOLTAGE_INTERVAL;
  }  
  
  if(lastMotorCommand + MOTOR_TIMEOUT < currentTime){
    resetMotor();
    lastMotorCommand = currentTime;
  }
  nh.spinOnce();
}
