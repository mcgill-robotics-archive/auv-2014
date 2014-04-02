/*
Maps thrust to voltage
Maps voltage to motor command
*/

#include "thrust_mapper.h"

//global vars
ros::Publisher voltage_publisher;
ros::Publisher thrust_publisher;
double VOLTAGE_MAX;
int32_t MOTOR_CMD_MAX;
double F_MAX;
double T_MAX;

float limit_check(float value, float max, char* value_type, char* value_id){
	if (value > max | value < -1*max) {
		ROS_WARN("%s: %s value is %f. This exceeds the maximum allowable value of %f.", value_type, value_id, value, max);

		value = 0; 

		return value;
	}
	else {
		return value;
	}

}


//update these values based on final characterization
float thrust_voltage(float thrust){
	float voltage;	
	    if (thrust<0.4641 & thrust>-0.01095 ) {
	        voltage = 0;
	    }
	    else if (thrust>0.4641 & thrust<0.79423) {
	        voltage = (-0.1419 + sqrt(0.1419*0.1419-4*0.0676*(-0.3668-thrust)))/(2*0.0676);
	    }
	    else if (thrust>0.79423) {
	    	voltage=(thrust+3.2786)/1.047;
	    }
	    else if (thrust<-0.01095 & thrust>-1.179701){
	        voltage = -0.0314 + sqrt(0.0314*0.0314-4*-0.0851*(0.0042-thrust));
	    }
	    else if (thrust<-1.179701) {
	    	voltage = (thrust-2.5582)/0.9609;
	    }
	    	return voltage;
}

void thrust_callback(geometry_msgs::Wrench wrenchMsg)
{	
	float thrust[6] = {0, 0, 0, 0, 0, 0};
	float voltage[6] = {0, 0, 0, 0, 0, 0};
	int32_t motor_cmd[6] = {0, 0, 0, 0, 0, 0};
	

	controls::motorCommands motorCommands;
	controls::DebugControls DebugControls;

	//Limit check for input wrench values
	wrenchMsg.force.x=limit_check(wrenchMsg.force.x, F_MAX, "Net Force", "X");
	wrenchMsg.force.y=limit_check(wrenchMsg.force.y, F_MAX, "Net Force", "Y");
	wrenchMsg.force.z=limit_check(wrenchMsg.force.z, F_MAX, "Net Force", "Z");
	wrenchMsg.torque.y=limit_check(wrenchMsg.torque.y, T_MAX, "Net Torque", "Y");
	wrenchMsg.torque.z=limit_check(wrenchMsg.torque.z, T_MAX, "Net Torque", "Z");

	//Account for geometry of the robot, thruster placement:
	// Coefficients are radii from center of rotation to thruster?
	thrust[0] = 0.5 * wrenchMsg.force.x; 
	thrust[1] = thrust[0];
	thrust[2] = 0.5 * wrenchMsg.force.y + 1.6667 * wrenchMsg.torque.z;
	thrust[3] = 0.5 * wrenchMsg.force.y - 1.6667 * wrenchMsg.torque.z;
	thrust[4] = 0.5 * wrenchMsg.force.z + 1.6667 * wrenchMsg.torque.y;
	thrust[5] = 0.5 * wrenchMsg.force.z - 1.6667 * wrenchMsg.torque.y;    

	
	//for each thrust, map to a voltage
	for (int i=0; i<6; i++) {
	}
	
	//Saturation of voltage values
	char* voltage_name[] {"one", "two", "three", "four", "five", "six"};
 	for (int i=0; i<6; i++)
	{
		voltage[i] = thrust_voltage(thrust[i]);
		voltage[i] = limit_check(voltage[i], VOLTAGE_MAX, "VOLTAGE", voltage_name[i]);
	}	

	//map voltages to motor commands
	//Conversion to integer between -500 and +500
	//linear for now TODO change mapping according to test

	char* motor_name[] {"one", "two", "three", "four", "five", "six"};
	for (int i=0; i<6; i++)
	{	
		motor_cmd[i]=21.93*voltage[i]-33.729; //TODO Take out of loop, use data from all motor controller characterization tests
		motor_cmd[i] = limit_check(motor_cmd[i], MOTOR_CMD_MAX, "MOTOR", motor_name[i]);//TODO Find way to pass which motor into string, create and loop through enumerator
	}

	motorCommands.cmd_x1=motor_cmd[0];
	motorCommands.cmd_x2=motor_cmd[1];
	motorCommands.cmd_y1=motor_cmd[2];
	motorCommands.cmd_y2=motor_cmd[3];
	motorCommands.cmd_z1=motor_cmd[4];
	motorCommands.cmd_z2=motor_cmd[5]; 

	DebugControls.thrust_surge_port=thrust[0];
	DebugControls.thrust_surge_starboard=thrust[1];
	DebugControls.thrust_sway_bow=thrust[2];
	DebugControls.thrust_sway_stern=thrust[3];
	DebugControls.thrust_heave_bow=thrust[4];
	DebugControls.thrust_heave_stern=thrust[5];


		//publish
	voltage_publisher.publish(motorCommands);
	thrust_publisher.publish(DebugControls);

}


int main(int argc, char **argv)
{
	// ROS subscriber setup
	ros::init(argc,argv,"thrust_mapper");
	ros::NodeHandle n;

	//Parameters
	n.param<double>("voltage/max", VOLTAGE_MAX, 0.0);
	n.param<int32_t>("motorCommands/max", MOTOR_CMD_MAX, 0.0);
	n.param<double>("force/max", F_MAX, 0.0);
	n.param<double>("torque/max", T_MAX, 0.0);

	//TODO raise warning if any of these variables == their default of 0. This means bad parameter file.
	if (VOLTAGE_MAX == 0.0){ROS_ERROR("PARAMETER FILE DID NOT LOAD IN THRUST_MAPPER");}
	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	//add clock subscription

	//ROS Publisher setup
	voltage_publisher = n.advertise<controls::motorCommands>("/controls/motorCommands", 100); //TODO change message type and name
	thrust_publisher = n.advertise<controls::DebugControls>("/controls/DebugControls", 100); //TODO change message type and name

	ROS_INFO("Thrust_mapper initialized. Listening for wrench.");
	ros::spin();
	return 0;
}
