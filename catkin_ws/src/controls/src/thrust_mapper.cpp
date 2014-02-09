/*

Maps thrust to voltage
Maps voltage to motor command

*/



#include "ros/ros.h"
#include "geometry_msgs/Wrench.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"
#include "controls/motorCommands.h"

//global vars
ros::Publisher voltage_publisher;

const float VOLTAGE_MIN=-20;
const float VOLTAGE_MAX=20;

void thrust_callback(const geometry_msgs::Wrench wrenchMsg)
{	
	float thrust[6] = {0, 0, 0, 0, 0, 0};
	float voltage[6] = {0, 0, 0, 0, 0, 0};
	int32_t motor_cmd[6] = {0, 0, 0, 0, 0, 0};
	
	const int32_t MOTOR_CMD_MAX = 127;
	const int32_t MOTOR_CMD_MIN = -128;

	controls::motorCommands motorCommands;

	ROS_DEBUG("Subscriber received wrench");

	thrust[0] = 0.5 * wrenchMsg.force.x;
	thrust[1] = thrust[0];
	thrust[2] = 0.5 * wrenchMsg.force.y + 1.6667 * wrenchMsg.torque.z;
	thrust[3] = 0.5 * wrenchMsg.force.y - 1.6667 * wrenchMsg.torque.z;
	thrust[4] = 0.5 * wrenchMsg.force.z + 1.6667 * wrenchMsg.torque.y;
	thrust[5] = 0.5 * wrenchMsg.force.z - 1.6667 * wrenchMsg.torque.y;    

	
	//for each thrust, map to a voltage
	for (int i=0; i<6; i++) {
	    if (thrust[i]<0.4641 & thrust[i]>-0.01095 ) {
	        voltage[i] = 0;
	    }
	    else if (thrust[i]>0.4641 & thrust[i]<0.79423) {
	        voltage[i] = (-0.1419 + sqrt(0.1419*0.1419-4*0.0676*(-0.3668-thrust[i])))/(2*0.0676);
	    }
	    else if (thrust[i]>0.79423) {
	    	voltage[i]=(thrust[i]+3.2786)/1.047;
	    }
	    else if (thrust[i]<-0.01095 & thrust[i]>-1.179701){
	        voltage[i] = -0.0314 + sqrt(0.0314*0.0314-4*-0.0851*(0.0042-thrust[i]));
	    }
	    else if (thrust[i]<-1.179701) {
	    	voltage[i] = (thrust[i]-2.5582)/0.9609;
	    }
	}
	//Add saturation statement, ROS_INFO("VALUE TOO GREAT")
	//Saturation of voltage values
	for (int i=0; i<6; i++) {
		voltage[i]=saturation(voltage[i], VOLTAGE_MIN, VOLTAGE_MAX, i);
	}


	//map voltages to motor commands
	//Conversion to integer between -128 and +127
	//linear for now TODO change mapping according to test
	for (int i=0; i<6; i++)
	{
		motor_cmd[i] = (voltage[i]-VOLTAGE_MIN)/(VOLTAGE_MAX-VOLTAGE_MIN)*(MOTOR_CMD_MAX-(MOTOR_CMD_MIN)) + (MOTOR_CMD_MIN); 
	}
	motorCommands.cmd_x1=motor_cmd[0];
	motorCommands.cmd_x2=motor_cmd[1];
	motorCommands.cmd_y1=motor_cmd[2];
	motorCommands.cmd_y2=motor_cmd[3];
	motorCommands.cmd_z1=motor_cmd[4];
	motorCommands.cmd_z2=motor_cmd[5]; 


		//publish
	voltage_publisher.publish(motorCommands);

}

float saturation(float value, float min, float max, int32_t counter){
	if (value > max | value < min) {
		value = 0;
	ROS_WARN("Saturation values have been exceeded. Thrust %i has been set to 0.", counter);
	}
	else {
		return value;
	}
}

int main(int argc, char **argv)
{
	// ROS subscriber setup
	ros::init(argc,argv,"thrust_mapper");
	ros::NodeHandle n;
	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	//add clock subscription

	//ROS Publisher setup
	voltage_publisher = n.advertise<geometry_msgs::Wrench>("/controls/motorCommands", 100); //TODO change message type and name
	
	ros::spin();
	return 0;
}
