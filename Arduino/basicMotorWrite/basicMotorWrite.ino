#include <Servo.h> 
//Pin definitions 

//PWM MOTOR
  #define MOTOR_PIN_X_1 3
  #define MOTOR_PIN_X_2 5
  #define MOTOR_PIN_Y_1 6
  #define MOTOR_PIN_Y_2 9
  #define MOTOR_PIN_Z_1 10
  #define MOTOR_PIN_Z_2 11

Servo myservo[6];

void setup(){
  myservo[0].attach(MOTOR_PIN_X_1);
  myservo[1].attach(MOTOR_PIN_X_2);
  myservo[2].attach(MOTOR_PIN_Y_1);
  myservo[3].attach(MOTOR_PIN_Y_2);
  myservo[4].attach(MOTOR_PIN_Z_1);
  myservo[5].attach(MOTOR_PIN_Z_2);
}

void loop(){
  
  int cmd_surge_starboard = 0;
  int cmd_surge_port = 0;
  int cmd_sway_stern = 0;
  int cmd_sway_bow = 0;
  int cmd_heave_bow = 0;
  int cmd_heave_stern = 0;
  
  /*
  int cmd_surge_starboard = 100;
  int cmd_surge_port = 100;
  int cmd_sway_stern = 100;
  int cmd_sway_bow = 100;
  int cmd_heave_bow = 100;
  int cmd_heave_stern = 100;
  */
  
  myservo[0].writeMicroseconds(1496 + (cmd_surge_starboard));
  myservo[1].writeMicroseconds(1499 + (cmd_surge_port));
  myservo[2].writeMicroseconds(1471 + (cmd_sway_bow));
  myservo[3].writeMicroseconds(1476 + (cmd_sway_stern));
  myservo[4].writeMicroseconds(1469 + (cmd_heave_bow));
  myservo[5].writeMicroseconds(1476 + (cmd_heave_stern));
}
