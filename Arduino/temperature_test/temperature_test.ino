#include <ros.h>
#include <std_msgs/Float32.h>

#define VOLT_RATIO 0.322265625
//Pin definitions 
  #define TEMP_PIN_1 A0
  #define TEMP_PIN_2 A1
  #define TEMP_PIN_3 A2
  #define TEMP_PIN_4 A3
  #define TEMP_PIN_5 A4
  
  
  
//TIME INTERVAL(unit microsecond)
  #define TEMPERATURE_INTERVAL 1000

ros::NodeHandle nh;
std_msgs::Float32 temperature1_msg;
std_msgs::Float32 temperature2_msg;
std_msgs::Float32 temperature3_msg;
std_msgs::Float32 temperature4_msg;
std_msgs::Float32 temperature5_msg;

unsigned long temperatureSechedule;
ros::Publisher temperature1_pub("/temperature1", &temperature1_msg);
ros::Publisher temperature2_pub("/temperature2", &temperature2_msg);
ros::Publisher temperature3_pub("/temperature3", &temperature3_msg);
ros::Publisher temperature4_pub("/temperature4", &temperature4_msg);
ros::Publisher temperature5_pub("/temperature5", &temperature5_msg);

void setup(){
  //ros node initialization
  nh.initNode();
  
  //ros publisher initialization
  nh.advertise(temperature1_pub);
  nh.advertise(temperature2_pub);
  nh.advertise(temperature3_pub);
  nh.advertise(temperature4_pub);
  nh.advertise(temperature5_pub);
  //ros subscribe initialization
}

void loop(){
  long currentTime = millis();
  
  //temperature and pressure sensing
  if(temperatureSechedule < currentTime){
   temperature1_msg.data=analogRead(TEMP_PIN_1) * VOLT_RATIO -50; 
   temperature1_pub.publish(&temperature1_msg);
   temperature2_msg.data=analogRead(TEMP_PIN_2) * VOLT_RATIO -50; 
   temperature2_pub.publish(&temperature2_msg);
   temperature3_msg.data=analogRead(TEMP_PIN_3) * VOLT_RATIO -50; 
   temperature3_pub.publish(&temperature3_msg);
   temperature4_msg.data=analogRead(TEMP_PIN_4) * VOLT_RATIO -50; 
   temperature4_pub.publish(&temperature4_msg);
   temperature5_msg.data=analogRead(TEMP_PIN_5) * VOLT_RATIO -50; 
   temperature5_pub.publish(&temperature5_msg);
   
   temperatureSechedule += TEMPERATURE_INTERVAL;
  }

  nh.spinOnce();
}
