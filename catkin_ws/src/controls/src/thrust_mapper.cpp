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
	/*
	* Input: desired thrust from a certain thruster
	* Output: voltage to send to that thruster
	*/

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
	/*
	* Input: net wrench to apply to robot
	* computes the desired thrust for each thruster
	* calculates the corresponding voltage via function call
	* calculates the corresponding motor command for these voltages
	*	-Motor controller characterization is coded here
	*	-offsets are implemented on the arduino (adding or subtracting yintercept)	
	* Output: motor commands saved in an array
	*/

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
	/*
		Coefficients are radii from center of rotation to thruster? Not sure about this...
			TODO WHERE TO THE COEFFICENTS COME FROM?????
		Propeller shroud is in the direction of positive force (send positive motor command and the robot will move in the direction of the shroud)
		Thruster layout as of April 14, 2014
			Surge thrusters positive forward
			Heave thrusters positive upward (should be switched though to avoid splashing at the surface)
			Sway-Bow points in the negative-y direction (applies a negative force and negative torque)
			Sway-Stern points in the positive-y direction (applies a positive force and negative torque)
		Order of thrust array
			0,1,2,3,4,5  :  x1,x2,y1,y2,z1,z2 :  surge-starbord, surge-port, sway-bow, sway-stern, heave-bow, heave-stern
	*/
	
	float mystery = 1.6667; //TODO figure out what this number is. I think it's a combo of the distnce between axes and the distribution across yaw thrusters (Actually I think it comes from inverting a matrix in matlab)
	int directions[] = {1, 1, -1, 1, -1, -1}; //the math below assumes thrusters apply force in their positive coordinate directions. -1 here if oriented otherwise

	thrust[0] = 0.5 * wrenchMsg.force.x; 										//surge-starbord
	thrust[1] = thrust[0];														//surge-port
	thrust[2] = 0.5 * wrenchMsg.force.y + mystery * wrenchMsg.torque.z;			//Sway-Bow
	thrust[3] = 0.5 * wrenchMsg.force.y - mystery * wrenchMsg.torque.z;			//sway-stern
	thrust[4] = 0.5 * wrenchMsg.force.z - mystery * wrenchMsg.torque.y;			//heave-bow  //sign between force and mystery swapped April 14, see nicks design notebok page 158
	thrust[5] = 0.5 * wrenchMsg.force.z + mystery * wrenchMsg.torque.y;    		//heave-stern  //sign between force and mystery swapped April 14, see nicks design notebok page 158
	
	for (int i=0; i<6; i++)
	{
		thrust[i]*=directions[i];
	}
	//for each thrust, map to a voltage, considering saturation
	char* voltage_name[] {"one", "two", "three", "four", "five", "six"};
 	for (int i=0; i<6; i++)
	{
		voltage[i] = thrust_voltage(thrust[i]);
		voltage[i] = limit_check(voltage[i], VOLTAGE_MAX, "VOLTAGE", voltage_name[i]);
		//ROS_INFO("Voltage %i: %f",i,voltage[i]);
	}	

	//map voltages to motor commands
		//Conversion to integer between -500 and +500
		//linear for now TODO change mapping according to motor controller characterization test

/*
	OLD THRUST MAPPING. MODIFIED RIGHT BEFORE TEST ON APRIL 3

	char* motor_name[] {"one", "two", "three", "four", "five", "six"};
	for (int i=0; i<6; i++)
	{	
		motor_cmd[i]=21.93*voltage[i]-33.729; //TODO Take out of loop, use data from all motor controller characterization tests
		motor_cmd[i] = limit_check(motor_cmd[i], MOTOR_CMD_MAX, "MOTOR", motor_name[i]);//TODO Find way to pass which motor into string, create and loop through enumerator
	}


*/
/*
	//less old voltage mapping. modified right before test on april 3

	motor_cmd[0] = 21.176*voltage[0] - 4.4494;
	motor_cmd[1] = 21.2*voltage[1] - 1.324;
	motor_cmd[2] = 20.686*voltage[2] - 28.9;
	motor_cmd[3] = 20.704*voltage[3] - 24.533;
	motor_cmd[4] = 20.583*voltage[4] - 30.652;
	motor_cmd[5] = 20.63*voltage[5] - 26.482;
*/

	// y intercepts moved because the offsets will be done on the arduino
	motor_cmd[0] = 21.176*voltage[0];
	motor_cmd[1] = 21.2*voltage[1];
	motor_cmd[2] = 20.686*voltage[2];
	motor_cmd[3] = 20.704*voltage[3];
	motor_cmd[4] = 20.583*voltage[4];
	motor_cmd[5] = 20.63*voltage[5];


	char* motor_name[] {"one", "two", "three", "four", "five", "six"};
	for (int i=0; i<6; i++)
	{	
		motor_cmd[i] = limit_check(motor_cmd[i], MOTOR_CMD_MAX, "MOTOR", motor_name[i]);//TODO Find way to pass which motor into string, create and loop through enumerator
	}

	motorCommands.cmd_surge_starboard=motor_cmd[0];
	motorCommands.cmd_surge_port=motor_cmd[1];
	motorCommands.cmd_sway_bow=motor_cmd[2];
	motorCommands.cmd_sway_stern=motor_cmd[3];
	motorCommands.cmd_heave_bow=motor_cmd[4];
	motorCommands.cmd_heave_stern=motor_cmd[5];

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
	voltage_publisher = n.advertise<controls::motorCommands>("/electrical_interface/motor", 100); 
	thrust_publisher = n.advertise<controls::DebugControls>("/controls/DebugControls", 100); 

	ROS_INFO("Thrust_mapper initialized. Listening for wrench.");
	ros::spin();
	return 0;
}
