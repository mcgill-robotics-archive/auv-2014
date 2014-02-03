#include <ros.h>
#include <std_msgs/Int16.h>

#define MAX_ANALOG 1023  // in bits
#define MAX_ARDUINO_VOLTAGE 5000 // in mV
#define RESISTANCE 100.0 // In ohms. Set to the value of the resistor being used. Should be less than 250 ohms because 5V / 20mA (max current) = 250 omhs.

#define BASE_CURRENT 4.0
#define CURRENT_RANGE 16.0   // from 4.0 mA to 20.0 mA
#define MAX_DEPTH 914.4      // In centimeters. 30 feet
#define OFFSET 7.0          // Just substract it. May need to be recalculated when circuit is built.

ros::NodeHandle nh;

std_msgs::Int16 depth_msg;
ros::Publisher depth("depth", &depth_msg);  // Publish the depth topic

int sensorPin = A0;      // select the input pin for the potentiometer
float sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
  nh.initNode();
  nh.advertise(depth);
}

void loop() {
  sensorValue = analogRead(sensorPin);                                             // ADC bits
  sensorValue = sensorValue*MAX_ARDUINO_VOLTAGE/(float)MAX_ANALOG;                 // From ADC to voltage in mV
  sensorValue = sensorValue/RESISTANCE;                                            // Get current in mA
  sensorValue = (int)(((sensorValue-BASE_CURRENT)/CURRENT_RANGE)*MAX_DEPTH);       // C0nvert the current to depth
  
  depth_msg.data = sensorValue - OFFSET;
  depth.publish(&depth_msg);
  nh.spinOnce();
  delay(100);
}
