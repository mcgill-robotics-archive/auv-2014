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
double VOLTAGE_MIN;
int32_t MOTOR_CMD_MAX;
int32_t MOTOR_CMD_MIN;
double F_MIN;
double F_MAX;
double T_MIN;
double T_MAX;

ros::NodeHandle n;

float limit_check(float value, float min, float max, char* string){
	if (value > max | value < min) {

		ROS_WARN("Limit values have been exceeded: %s. Value is %f", string, value);

		value = 0;

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
	
	double VOLTAGE_MAX;
	n.param<double>("voltage/max", VOLTAGE_MAX, 0.0);

	controls::motorCommands motorCommands;

	//Limit check for input wrench values
	wrenchMsg.force.x=limit_check(wrenchMsg.force.x, F_MIN, F_MAX, "Force X");
	wrenchMsg.force.y=limit_check(wrenchMsg.force.y, F_MIN, F_MAX, "Force Y");
	wrenchMsg.force.z=limit_check(wrenchMsg.force.z, F_MIN, F_MAX, "Force Z");
	wrenchMsg.torque.y=limit_check(wrenchMsg.torque.y, T_MIN, T_MAX, "Torque Y");
	wrenchMsg.torque.z=limit_check(wrenchMsg.torque.z, T_MIN, T_MAX, "Torque Z");



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



	//map voltages to motor commands
	//Conversion to integer between -128 and +127
	//linear for now TODO change mapping according to test
	for (int i=0; i<6; i++)
	{
		motor_cmd[i] = (voltage[i]-VOLTAGE_MIN)/(VOLTAGE_MAX-VOLTAGE_MIN)*(MOTOR_CMD_MAX-(MOTOR_CMD_MIN)) + (MOTOR_CMD_MIN);
	//Apply limit check to motor command 
		motor_cmd[i] = limit_check(motor_cmd[i], MOTOR_CMD_MIN, MOTOR_CMD_MAX, "MOTOR");//Find way to pass which motor into string, create and loop through enumerator

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
	//ros::NodeHandle n;

	//Parameters
	n.param<double>("voltage/min", VOLTAGE_MIN, 0.0);
	n.param<int32_t>("motorCommands/min", MOTOR_CMD_MIN, 0.0);
	n.param<int32_t>("motorCommands/max", MOTOR_CMD_MAX, 0.0);
	n.param<double>("force/min", F_MIN, 0.0);
	n.param<double>("force/max", F_MAX, 0.0);
	n.param<double>("torque/min", T_MIN, 0.0);
	n.param<double>("torque/max", T_MAX, 0.0);


	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	//add clock subscription

	//ROS Publisher setup
	voltage_publisher = n.advertise<controls::motorCommands>("/controls/motorCommands", 100); //TODO change message type and name
	
	ros::spin();
	return 0;
}
