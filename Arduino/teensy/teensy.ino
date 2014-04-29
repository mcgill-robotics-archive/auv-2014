#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>
#include <controls/motorCommands.h>
#include <arduino_msgs/solenoid.h>

//Pin definitions 

  //PWM MOTOR
  #define MOTOR_PIN_SU_ST 0
  #define MOTOR_PIN_SU_PO 1
  #define MOTOR_PIN_SW_BO 2
  #define MOTOR_PIN_SW_ST 3
  #define MOTOR_PIN_HE_BO 4
  #define MOTOR_PIN_HE_ST 5

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
  #define GRABBER_SWTCH_PIN_1 A4
  #define GRABBER_SWTCH_PIN_2 A5
  #define TEMPERATURE_PIN_1 A6
  #define TEMPERATURE_PIN_2 A7
  #define TEMPERATURE_PIN_3 A8
  #define TEMPERATURE_PIN_4 A9
  
//TIME INTERVAL(unit microsecond)
  #define MOTOR_TIMEOUT 4000          //amount of no signal required to start to reset motors 
  #define TEMPERATURE_INTERVAL 1000   //amount of delay between each temeperatures read
  #define VOLTAGE_INTERVAL 1000       //amount of delay between each voltages read
  #define DEPTH_INTERVAL 200          //amount of delay between each depth read

ros::NodeHandle nh;
std_msgs::Int16 depth_msg;
std_msgs::Float32 batteryVoltage1_msg;
std_msgs::Float32 batteryVoltage2_msg;
std_msgs::Float32 temperature1_msg;
std_msgs::Float32 temperature2_msg;
std_msgs::Float32 temperature3_msg;
std_msgs::Float32 temperature4_msg;

Servo myservo[6];

unsigned long depthSensorSchedule;
unsigned long batteryVoltageSchedule;
unsigned long temperatureSechedule;
unsigned long lastMotorCommand;

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
  myservo[0].writeMicroseconds(1496 + boundCheck(msg.cmd_surge_starboard));
  myservo[1].writeMicroseconds(1499 + boundCheck(msg.cmd_surge_port));
  myservo[2].writeMicroseconds(1471 + boundCheck(msg.cmd_sway_bow));
  myservo[3].writeMicroseconds(1476 + boundCheck(msg.cmd_sway_stern));
  myservo[4].writeMicroseconds(1469 + boundCheck(msg.cmd_heave_bow));
  myservo[5].writeMicroseconds(1476 + boundCheck(msg.cmd_heave_stern));
}

void resetMotor(){
  myservo[0].writeMicroseconds(1496);
  myservo[1].writeMicroseconds(1499);
  myservo[2].writeMicroseconds(1471);
  myservo[3].writeMicroseconds(1476);
  myservo[4].writeMicroseconds(1469);
  myservo[5].writeMicroseconds(1476);
}



void solenoidCb( const arduino_msgs::solenoid& msg){
  digitalWrite(SOLENOID_PIN_T_1,msg.torpedo1.data);
  digitalWrite(SOLENOID_PIN_T_2,msg.torpedo2.data);
  digitalWrite(SOLENOID_PIN_G_1,msg.grabber1.data);
  digitalWrite(SOLENOID_PIN_G_2,msg.grabber2.data);
  digitalWrite(SOLENOID_PIN_D_1,msg.dropper1.data);
  digitalWrite(SOLENOID_PIN_D_2,msg.dropper2.data);
}


ros::Publisher depthPub("/depth", &depth_msg);  // Publish the depth topic

ros::Publisher voltagePub1("/batteryVoltage1", &batteryVoltage1_msg);
ros::Publisher voltagePub2("/batteryVoltage2", &batteryVoltage2_msg);

ros::Publisher temperaturePub1("/temperature1", &temperature1_msg);
ros::Publisher temperaturePub2("/temperature2", &temperature2_msg);
ros::Publisher temperaturePub3("/temperature3", &temperature3_msg);
ros::Publisher temperaturePub4("/temperature4", &temperature4_msg);

ros::Subscriber<arduino_msgs::solenoid> solenoidSub("/solenoid", &solenoidCb );

ros::Subscriber<controls::motorCommands> motorSub("/motor", &motorCb );


void setup(){
  myservo[0].attach(MOTOR_PIN_SU_ST);
  myservo[1].attach(MOTOR_PIN_SU_PO);
  myservo[2].attach(MOTOR_PIN_SW_BO);
  myservo[3].attach(MOTOR_PIN_SW_ST);
  myservo[4].attach(MOTOR_PIN_HE_BO);
  myservo[5].attach(MOTOR_PIN_HE_ST);
  
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
  nh.advertise(temperaturePub1);
  nh.advertise(temperaturePub2);
  nh.advertise(temperaturePub3);
  nh.advertise(temperaturePub4);
  
  //ros subscribe initialization
  nh.subscribe(motorSub);  
  nh.subscribe(solenoidSub);

  resetMotor();

}

void loop(){
  
  long currentTime = millis();

  //temperature sensing
  if(temperatureSechedule < currentTime){
   temperature1_msg.data = analogRead(TEMPERATURE_PIN_1);
   temperaturePub1.publish(&temperature1_msg);
   temperature2_msg.data = analogRead(TEMPERATURE_PIN_2);
   temperaturePub2.publish(&temperature2_msg);
   temperature3_msg.data = analogRead(TEMPERATURE_PIN_3);
   temperaturePub3.publish(&temperature3_msg);
   temperature4_msg.data = analogRead(TEMPERATURE_PIN_4);
   temperaturePub4.publish(&temperature4_msg);
   temperatureSechedule += TEMPERATURE_INTERVAL;
  }
  
  //depth sensing  
  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(DEPTH_SENSOR_PIN);
    depthPub.publish(&depth_msg);
    depthSensorSchedule += DEPTH_INTERVAL;   
  }

  //voltages sensing
  if(batteryVoltageSchedule < currentTime){
    batteryVoltage1_msg.data = analogRead(VOLTAGE_PIN_1)*0.0341796875;
    batteryVoltage2_msg.data = analogRead(VOLTAGE_PIN_2)*0.0341796875;
    
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
