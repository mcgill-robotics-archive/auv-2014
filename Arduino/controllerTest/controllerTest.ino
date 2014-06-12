#include <Servo.h> 
Servo ser1;
Servo ser2;

void setup(){
ser1.attach(2);
ser2.attach(3);
Serial.begin(9600);
ser1.writeMicroseconds(1500);
ser2.writeMicroseconds(1500);
}

void loop(){
int temp = analogRead(A0);
Serial.println(String(temp) + "\t" +String(temp-512));
ser1.writeMicroseconds(982+temp);
ser2.writeMicroseconds(982+temp);
delay(200);
}
