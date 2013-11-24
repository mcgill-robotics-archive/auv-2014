//Sensor driver for sparkfun 9-DOF sensor stick

//I2C device found at address 0x1E
//I2C device found at address 0x53
//I2C device found at address 0x68 
#include <Wire.h>
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>

#define ACC_ADDRESS (0x53)
#define ACC_ID (B11100101)
#define ACC_REG_PWRCTL (0x2D)
#define ACC_PWRCTL_BIT (1<<3)
#define ACC_REG_DATA (0x32)
#define ACC_UNITS_FACTOR (9.8/256.0)

#define GYRO_ADDRESS (0x68)
#define GYRO_ID (B01101000)
#define GYRO_ID_MASK (B01111110)
#define GYRO_REG_DATA (0x1D)
#define GYRO_REG_RATE (0x15)
#define GYRO_RATE_DIVIDER (4) //frequency is given by (1kHz)/(divider+1) except when GYRO_DLPF_CFG
#define GYRO_REG_DLPF_FS (0x16)
#define GYRO_DLPF_CFG (1)
#define GYRO_FS_CFG (3 << 3)
#define GYRO_UNITS_FACTOR (3.14159/(14.375*180.0))

ros::NodeHandle nh;
geometry_msgs::Vector3Stamped acc_msg;
ros::Publisher acc_topic("acc_data", &acc_msg);
geometry_msgs::Vector3Stamped gyro_msg;
ros::Publisher gyro_topic("gyro_data", &gyro_msg);


byte accData[6];
int accMeasurement[3];
int accSign[] = {-1,1,1};

byte gyroData[6];
int gyroMeasurement[3];
int gyroSign[] = {-1,1,1};

void setup() {
  Wire.begin();
  //Serial.begin(115200);
  
  testAccConnection();
  initAcc();
  
  testGyroConnection();
  initGyro();
  
  nh.initNode();
  nh.advertise(acc_topic);
  nh.advertise(gyro_topic);
  
  //Serial.println("Measuring");
}


void loop() {
  delay(20);
  
  measureAcc();
  measureGyro();
  acc_topic.publish(&acc_msg);
  gyro_topic.publish(&gyro_msg);
  nh.spinOnce();
}

void measureGyro(){
  i2c_read(GYRO_ADDRESS, GYRO_REG_DATA,6,gyroData);
  
  //unpack
  gyro_msg.header.stamp = nh.now();
  gyro_msg.vector.x = GYRO_UNITS_FACTOR*gyroSign[0]*((int)gyroData[1] + (((int)gyroData[0]) << 8));
  gyro_msg.vector.y = GYRO_UNITS_FACTOR*gyroSign[1]*((int)gyroData[3] + (((int)gyroData[2]) << 8));
  gyro_msg.vector.z = GYRO_UNITS_FACTOR*gyroSign[2]*((int)gyroData[5] + (((int)gyroData[4]) << 8));
  //Serial.println(gyroMeasurement[i]);
  
}

void initGyro() {
  //Set the frequency of ADC conversions
  if (! i2c_writeAndCheck(GYRO_ADDRESS, GYRO_REG_RATE, GYRO_RATE_DIVIDER))
  {
    //Serial.println("Could not initialize gyro");
    while(1);
  }
  
  //Set the DLPF and turn on
  if (i2c_writeAndCheck(GYRO_ADDRESS, GYRO_REG_DLPF_FS, GYRO_DLPF_CFG | GYRO_FS_CFG)) {
    //Serial.println("Gyro initialized!");
  }else{
    //Serial.println("Couldn't initialize gyro");
    while(1);
  }
  
}

void testGyroConnection(){
  if(isConnected(GYRO_ADDRESS, GYRO_ID, GYRO_ID_MASK)) {
    //Serial.println("Gyro connected");
  }else {
    //Serial.println("Gyro not found");
    while(1);
  }
}


void measureAcc(){
  //read the Accel data
  i2c_read(ACC_ADDRESS, ACC_REG_DATA, 6, accData);
  //unpack
  acc_msg.header.stamp = nh.now();
  acc_msg.vector.x = ACC_UNITS_FACTOR*accSign[0]*((int)accData[0] + (((int)accData[1])<<8));
  acc_msg.vector.y = ACC_UNITS_FACTOR*accSign[1]*((int)accData[2] + (((int)accData[3])<<8));
  acc_msg.vector.z = ACC_UNITS_FACTOR*accSign[2]*((int)accData[4] + (((int)accData[5])<<8));
}

void testAccConnection() {
  if (isConnected(ACC_ADDRESS, ACC_ID)) {
    //Serial.println("Accelerometer connected");
  }else {
    //Serial.println("Accelerometer not found");
    while(1);
  }
}

boolean initAcc() {
  byte data = 0;
  //Need to set the device to measure mode
  i2c_write(ACC_ADDRESS, ACC_REG_PWRCTL, ACC_PWRCTL_BIT);
  
  //Check the change stuck
  i2c_read(ACC_ADDRESS, ACC_REG_PWRCTL, 1, &data);
  
  if (data == ACC_PWRCTL_BIT) {
    //Serial.println("Accelerometer initialized");
  } else {
    //Serial.println("Could not initialize accelerometer");
    while(1);
  }
}  

boolean isConnected(int address, byte devID) {
  isConnected(address,devID,B11111111);
}

boolean isConnected(int address, byte devID, byte IDmask) {
  byte data = 0;
  Wire.beginTransmission(address);
  if (Wire.endTransmission() != 0)
    return false;
    
  i2c_read(address,0,1,&data);
  return (data & IDmask) == devID;
}

boolean i2c_writeAndCheck(int address, byte reg, byte dataIn){
  byte dataOut = 0;
  
  i2c_write(address,reg,dataIn);
  i2c_read(address,reg,1,&dataOut);
  
  return dataOut==dataIn;
}

void i2c_write(int address, byte reg, byte data) {
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.write(data);
  Wire.endTransmission();
}

void i2c_read(int address, byte reg, int count, byte* data) {
  //Select register
  Wire.beginTransmission(address);
  Wire.write(reg);
  Wire.endTransmission();
  
  Wire.requestFrom(address, count);
  for (int i = 0; i<count; i++) {
    if (Wire.available()) {
      data[i] = Wire.read();
    }else {
      Serial.println("Could not read device");
    }
  }
}
