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
#include <string>

//global vars
ros::Publisher voltage_publisher;
double VOLTAGE_MAX;
int32_t MOTOR_CMD_MAX;
int32_t MOTOR_CMD_MIN;
double F_MAX;
double T_MAX;
double motor__cmd_slope
double motor_cmd_intersect

float limit_check(float value, float max, char* value_type, char* value_id ){
	if (value > max | value < -1*max) {

		ROS_WARN("%s: %s value has been exceeded. Value is %f", value_type, value_id, value);

		value = 0;
		return value;

	}
	else {
		return value;
	}
}


void thrust_callback(geometry_msgs::Wrench wrenchMsg)
{	
	float thrust[6] = {0, 0, 0, 0, 0, 0};
	float voltage[6] = {0, 0, 0, 0, 0, 0};
	int32_t motor_cmd[6] = {0, 0, 0, 0, 0, 0};
	

	controls::motorCommands motorCommands;

	//Limit check for input wrench values
	wrenchMsg.force.x=limit_check(wrenchMsg.force.x, F_MAX, "Net Force", "X");
	wrenchMsg.force.y=limit_check(wrenchMsg.force.y, F_MAX, "Net Force", "Y");
	wrenchMsg.force.z=limit_check(wrenchMsg.force.z, F_MAX, "Net Force", "Z");
	wrenchMsg.torque.y=limit_check(wrenchMsg.torque.y, T_MAX, "Net Torque", "Y");
	wrenchMsg.torque.z=limit_check(wrenchMsg.torque.z, T_MAX, "Net Torque", "Z");



	ROS_DEBUG("Subscriber received wrench");
	//Account for geometry of the robot, thruster placement:
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
	
	//Saturation of voltage values
	char* voltage_name[] {"one", "two", "three", "four", "five", "six"};
 	for (int i=0; i<6; i++)
	{
		voltage[i] = limit_check(voltage[i], VOLTAGE_MAX, "VOLTAGE", voltage_name[i]);

	}	

	

	//map voltages to motor commands
	//Conversion to integer between -500 and +500
	//linear for now TODO change mapping according to test
	char* motor_name[] {"one", "two", "three", "four", "five", "six"};
	for (int i=0; i<6; i++)
	{	
		motor_cmd[i] = limit_check(motor_cmd[i], MOTOR_CMD_MIN, MOTOR_CMD_MAX, "MOTOR", motor_name[i]);//TODO Find way to pass which motor into string, create and loop through enumerator
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



int main(int argc, char **argv)
{
	// ROS subscriber setup
	ros::init(argc,argv,"thrust_mapper");
	ros::NodeHandle n;

	//Parameters
	n.param<double>("voltage/max", VOLTAGE_MAX, 0.0);
	n.param<double>("voltage/min", VOLTAGE_MIN, 0.0);
	n.param<int32_t>("motorCommands/min", MOTOR_CMD_MIN, 0.0);
	n.param<int32_t>("motorCommands/max", MOTOR_CMD_MAX, 0.0);
	n.param<double>("force/max", F_MAX, 0.0);
	n.param<double>("torque/max", T_MAX, 0.0);

	//TODO raise warning if any of these variables == their default of 0. This means bad parameter file.

	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	//add clock subscription

	//ROS Publisher setup
	voltage_publisher = n.advertise<controls::motorCommands>("/controls/motorCommands", 100); //TODO change message type and name
	
	ros::spin();
	return 0;
}
