#include <Servo.h> 
#include <ros.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <controls/motorCommands.h>
#include <arduino_msgs/solenoid.h>

//Pin definitions 

//PWM MOTOR
  #define MOTOR_PIN_X_1 3
  #define MOTOR_PIN_X_2 5
  #define MOTOR_PIN_Y_1 6
  #define MOTOR_PIN_Y_2 9
  #define MOTOR_PIN_Z_1 10
  #define MOTOR_PIN_Z_2 11

//SOLENOID
  #define SOLENOID_PIN_D_1 2 
  #define SOLENOID_PIN_D_2 4
  #define SOLENOID_PIN_G_1 7
  #define SOLENOID_PIN_G_2 8
  #define SOLENOID_PIN_T_1 12
  #define SOLENOID_PIN_T_2 13

//ANALOG
  #define DEPTH_SENSOR_PIN A0
  #define GRABBER_SWTCH_PIN A1
  #define VOLTAGE_PIN_1 A2
  #define VOLTAGE_PIN_2 A3

//SENSING INTERVAL(unit microsecond)
  #define VOLTAGE_INTERVAL 1000
  #define PRESSURE_INTERVAL 200
  #define DEPTH_INTERVAL 1000

ros::NodeHandle nh;
std_msgs::Int16 depth_msg;
std_msgs::Int16 batteryVoltage1_msg;
std_msgs::Int16 batteryVoltage2_msg;
std_msgs::Int16 temperature_msg;
std_msgs::Int32 pressure_msg;

Servo myservo[6];

unsigned long depthSensorSchedule;
unsigned long batteryVoltageSchedule;
unsigned long temperaturePressureSechedule;


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
ros::Publisher temperaturePub("/temperature", &temperature_msg);
ros::Publisher pressurePub("/pressure", &pressure_msg);
ros::Subscriber<arduino_msgs::solenoid> solenoidSub("/solenoid", &solenoidCb );
ros::Subscriber<controls::motorCommands> motorSub("/motor", &motorCb );
void setup(){
  myservo[0].attach(MOTOR_PIN_X_1);
  myservo[1].attach(MOTOR_PIN_X_2);
  myservo[2].attach(MOTOR_PIN_Y_1);
  myservo[3].attach(MOTOR_PIN_Y_2);
  myservo[4].attach(MOTOR_PIN_Z_1);
  myservo[5].attach(MOTOR_PIN_Z_2);
  
  pinMode(SOLENOID_PIN_T_1,OUTPUT);
  pinMode(SOLENOID_PIN_T_2,OUTPUT);
  pinMode(SOLENOID_PIN_G_1,OUTPUT);
  pinMode(SOLENOID_PIN_G_2,OUTPUT);
  pinMode(SOLENOID_PIN_D_1,OUTPUT);
  pinMode(SOLENOID_PIN_D_2,OUTPUT);

    
  //ros node initialization
  nh.initNode();
  
  //ros publisher initialization
  nh.advertise(depthPub);        //depth sensor
  nh.advertise(voltagePub1);     //battery level
  nh.advertise(voltagePub2);
  nh.advertise(temperaturePub);
  nh.advertise(pressurePub);
  
  //ros subscribe initialization
  nh.subscribe(motorSub);  
  nh.subscribe(solenoidSub);

  //BMP085 Setup
  bmp085Calibration();

}

void loop(){
  
  long currentTime = millis();

  //temperature and pressure sensing
  if(temperaturePressureSechedule < currentTime){
   pressure_msg.data= bmp085GetPressure(bmp085ReadUP());
   temperature_msg.data = bmp085GetTemperature(bmp085ReadUT());
   temperaturePub.publish(&temperature_msg);
   pressurePub.publish(&pressure_msg);
   temperaturePressureSechedule += PRESSURE_INTERVAL;
  }
  
  //depth sensing  
  if(depthSensorSchedule < currentTime){
    depth_msg.data = analogRead(DEPTH_SENSOR_PIN);
    depthPub.publish(&depth_msg);
    depthSensorSchedule += DEPTH_INTERVAL;        //Update at 5Hz  
  }

  //voltages sensing
  if(batteryVoltageSchedule < currentTime){
    batteryVoltage1_msg.data = analogRead(VOLTAGE_PIN_1);
    batteryVoltage2_msg.data = analogRead(VOLTAGE_PIN_2);
    
    voltagePub1.publish(&batteryVoltage1_msg);
    voltagePub2.publish(&batteryVoltage2_msg);
    
    batteryVoltageSchedule += VOLTAGE_INTERVAL;     //Update at 1 Hz
  }  
  nh.spinOnce();
}
