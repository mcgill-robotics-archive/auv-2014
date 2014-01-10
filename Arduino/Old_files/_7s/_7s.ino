#include <ros.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Vector3.h>
ros::NodeHandle nh;
void setLed(const std_msgs::Int16& value){
PORTA=seg(value.data);
}
ros::Subscriber<std_msgs::Int16> led("value", &setLed);
geometry_msgs::Vector3 acceleration;
ros::Publisher report("accmeter/acc",&acceleration);

int seg(int index){
switch (index) {
    case 0:
      return B00001010;
    case 1:
      return B11101011;
    case 2:
      return B01000110;
    case 3:
      return B01100010;
    case 4:
      return B10100011;
    case 5:
      return B00110010;
    case 6:
      return B00010010;
    case 7:
      return B01101011;
    case 8:
      return B00000010;
    case 9:
      return B00100010;
    case 10:
      return B00000011;
    case 11:
      return B10010010;
    case 12:
      return B00011110;
    case 13:
      return B11000010;
    case 14:
      return B00010110;
    case 15:
      return B00010111;  
    default: 
      return B11111111;
  }


}

void setup(){
DDRA= B11111111;
nh.initNode();
nh.subscribe(led);
nh.advertise(report);

}
void loop(){
  nh.spinOnce();
  delay(600);
}
