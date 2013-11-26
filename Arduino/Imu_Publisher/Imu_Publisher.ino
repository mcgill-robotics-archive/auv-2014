#include <ros.h>
#include <geometry_msgs/Vector3.h>

#include <Wire.h>

#define MPU9150_ADDRESS 0x68	   // MPU9150 address (when AD0 is LOW)

// Accelerometer readings
#define MPU9150_ACCEL_CONFIG 0x1C  // Accelerometer configuration register
#define MPU9150_ACCEL_XOUT_H 0x3B  // Accelerometer x MSB register
#define MPU9150_ACCEL_XOUT_L 0x3C  // Accelerometer x LSB register
#define MPU9150_ACCEL_YOUT_H 0x3D  // Accelerometer y MSB register
#define MPU9150_ACCEL_YOUT_L 0x3E  // Accelerometer y LSB register
#define MPU9150_ACCEL_ZOUT_H 0x3F  // Accelerometer z MSB register
#define MPU9150_ACCEL_ZOUT_L 0x40  // Accelerometer z LSB register

// Gyroscope readings
#define MPU9150_GYRO_CONFIG 0x1B  // Gyroscope configuration register
#define MPU9150_GYRO_XOUT_H 0x43  // Gyroscope x MSB register
#define MPU9150_GYRO_XOUT_L 0x44  // Gyroscope x LSB register
#define MPU9150_GYRO_YOUT_H 0x45  // Gyroscope y MSB register
#define MPU9150_GYRO_YOUT_L 0x46  // Gyroscope y LSB register
#define MPU9150_GYRO_ZOUT_H 0x47  // Gyroscope z MSB register
#define MPU9150_GYRO_ZOUT_L 0x48  // Gyroscope z LSB register

#define MPU9150_PWR_MGMT_1 0x6B    // Power mode register

#define MIN_INT -32768             // Smallest value in 16-bit integers
#define MAX_INT +32767             // Largest value in 16-bit integers  

// Accelerometer range & sensitivity
int MPU9150_AFS_SEL = 0;                                                           // 0, 1, 2 or 3.                                                        
const double MPU9150_ACCEL_RANGE[4] = { 2 * 9.81, 4 * 9.81, 6 * 9.81, 8 * 9.81 };  // in m/s^2
  
// Gyroscope range & sensitivity
int MPU9150_FS_SEL = 0;                                                            // 0, 1, 2 or 3.
const double MPU9150_GYRO_RANGE[4] = { 250.0, 500.0, 1000.0, 2000.0 };             // in deg/s

ros::NodeHandle nh;  // Handle needed to communicate with ROS

// Messages
geometry_msgs::Vector3 accel_msg;          // To publish in the accel topic (linear acceleration in m/s^2)
geometry_msgs::Vector3 gyro_msg;           // To publish in the gyro topic (angular velocity in deg/s)

// Publishers
ros::Publisher accel("accel", &accel_msg);       // Publish the accel topic
ros::Publisher gyro("gyro", &gyro_msg);          // Publish the gyro topic

// Function prototypes
void MPU9150_getAccelReadings(geometry_msgs::Vector3 &);
void MPU9150_getGyroReadings(geometry_msgs::Vector3 &);
void MPU9150_setAccelRange(int);
void MPU9150_setGyroRange(int);
int MPU9150_readSensor(int);
int MPU9150_readSensor(int, int);
void MPU9150_writeSensor(int, int);
double map_float(double, double, double, double, double);

void setup()
{
    Wire.begin();        // join i2c bus (address optional for master)
    
    MPU9150_writeSensor(MPU9150_PWR_MGMT_1,0);     // Turn off sleep mode
    MPU9150_setAccelRange(0);                      // Set accelerometer range
    MPU9150_setGyroRange(1);                       // Set gyroscope range
    
    // ROS node initialization and advertise the topics
    nh.initNode();
    nh.advertise(accel);
    nh.advertise(gyro);
}

void loop()
{ 
    // Populate messages
    MPU9150_getAccelReadings(accel_msg);
    MPU9150_getGyroReadings(gyro_msg);
    
    // Publish messages
    accel.publish(&accel_msg);
    nh.spinOnce();
    gyro.publish(&gyro_msg);
    nh.spinOnce();
}

void MPU9150_getAccelReadings(geometry_msgs::Vector3 &accel_vector)
{
    double range = MPU9150_ACCEL_RANGE[MPU9150_AFS_SEL];  // The range of accelerations as set by AFS_SEL
    
    // Get the raw values from the accelerometer
    int ax = MPU9150_readSensor(MPU9150_ACCEL_XOUT_L, MPU9150_ACCEL_XOUT_H);
    int ay = MPU9150_readSensor(MPU9150_ACCEL_YOUT_L, MPU9150_ACCEL_YOUT_H);
    int az = MPU9150_readSensor(MPU9150_ACCEL_ZOUT_L, MPU9150_ACCEL_ZOUT_H);
    
    // Scale the values by mapping into the range (from -range to +range)
    // MIN_INT and MAX_INT are used to find the domain of the raw data
    accel_vector.x = map_float( ax, MIN_INT, MAX_INT, -range, +range );
    accel_vector.y = map_float( ay, MIN_INT, MAX_INT, -range, +range );
    accel_vector.z = map_float( az, MIN_INT, MAX_INT, -range, +range );
}

void MPU9150_getGyroReadings(geometry_msgs::Vector3 &gyro_vector)
{
    double range = MPU9150_GYRO_RANGE[MPU9150_FS_SEL];  // The range of angular velocities as set by FS_SEL
    
    // Get the raw values from the gyroscope
    int gx = MPU9150_readSensor(MPU9150_GYRO_XOUT_L, MPU9150_GYRO_XOUT_H);
    int gy = MPU9150_readSensor(MPU9150_GYRO_YOUT_L, MPU9150_GYRO_YOUT_H);
    int gz = MPU9150_readSensor(MPU9150_GYRO_ZOUT_L, MPU9150_GYRO_ZOUT_H);
    
    // Scale the values by mapping into the range (from -range to +range)
    // MIN_INT and MAX_INT are used to find the domain of the raw data
    gyro_vector.x = map_float( gx, MIN_INT, MAX_INT, -range, +range );
    gyro_vector.y = map_float( gy, MIN_INT, MAX_INT, -range, +range );
    gyro_vector.z = map_float( gz, MIN_INT, MAX_INT, -range, +range );
}
    
void MPU9150_setAccelRange(int AFS_SEL)
{
    int configReg = MPU9150_readSensor(MPU9150_ACCEL_CONFIG); // get previous config value
    
    configReg &= B11100111;                                   // clear bits 4 and 3 (AFS_SEL)
    configReg |= (AFS_SEL<<3);                                // set AFS_SEL (left-shift by 3)
    MPU9150_writeSensor(MPU9150_ACCEL_CONFIG, configReg);
    MPU9150_AFS_SEL = AFS_SEL;
}
    
void MPU9150_setGyroRange(int FS_SEL)
{
    int configReg = MPU9150_readSensor(MPU9150_GYRO_CONFIG);  // get previous config value
    
    configReg &= B11100111;                                   // clear bits 4 and 3 (FS_SEL)
    configReg |= (FS_SEL<<3);                                 // set FS_SEL (left-shift by 3)
    MPU9150_writeSensor(MPU9150_GYRO_CONFIG, configReg);
    MPU9150_FS_SEL = FS_SEL;
}

int MPU9150_readSensor(int addr)
{   
    Wire.beginTransmission(MPU9150_ADDRESS);
    Wire.write(addr);                         // Address of the register
    Wire.endTransmission();
    
    Wire.requestFrom(MPU9150_ADDRESS,1);
    return Wire.read();                        // Read the value of register
}

int MPU9150_readSensor(int addrL, int addrH)  // When data is stored in 2 registers
{
    int low = MPU9150_readSensor(addrL);
    int high = MPU9150_readSensor(addrH);
    return ( high << 8 ) + low;              // combine high and low in int.
}

void MPU9150_writeSensor(int addr, int data)
{
    Wire.beginTransmission(MPU9150_ADDRESS);  
    Wire.write(addr);                          // Address in which to write
    Wire.write(data);                          // Write data in addr
    Wire.endTransmission();
}

double map_float(double val, double fromMin, double fromMax, double toMin, double toMax)
{
    if ( fromMax - fromMin <= 0.00000001 )
      return (toMin+toMax)/2;	// If the domain is too small, hard to tell where to map the value...
      
    return toMin + ( ( val - fromMin ) / ( fromMax - fromMin ) ) * ( toMax - toMin );
}
