/*

Maps thrust to voltage
Maps voltage to motor command

*/



#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "std_msgs/Float64.h"
#include "gazebo_msgs/ModelStates.h"
#include "controls/motorCommands.h"

void thrust_callback(const geometry_msgs::Wrench wrenchMsg)
{	
	float thrust[6] = [0, 0, 0, 0, 0, 0];
	float voltage[6] = [0, 0, 0, 0, 0, 0];
	int32_t motor_cmd[6] = [0, 0, 0, 0, 0, 0];
	
	const float VOLTAGE_MAX = 20;
	const float VOLTAGE_MIN = -20;

	const int MOTOR_CMD_MAX = 127;
	const int MOTOR_CMD_MIN = -128;

	controls::motorCommands motorCommands;

	ROS_DEBUG("Subscriber received wrench");

	thrust[0] = 0.5 * wrenchMsg.force.x;
	thrust[1] = thrust[0];
	thrust[2] = 0.5 * wrenchMsg.force.y + 1.6667 * wrenchMsg.torque.z;
	thrust[3] = 0.5 * wrenchMsg.force.y - 1.6667 * wrenchMsg.torque.z;
	thrust[4] = 0.5 * wrenchMsg.force.z + 1.6667 * wrenchMsg.torque.y;
	thrust[5] = 0.5 * wrenchMsg.force.z - 1.6667 * wrenchMsg.torque.y;    

	//for each thrust, map to a voltage
	for (int i=0; n<6; n++) {
	    if (thrust[i]<0.4641 AND thrust[i]>-0.01095 ) {
	        voltage[i] = 0;
	    }
	    else if (thrust[i]>0.4641 AND thrust[i]<0.79423) {
	        voltage[i] = (-0.1419 + sqrt(0.1419*0.1419-4*0.0676*(-0.3668-thrust[i])))/(2*0.0676);
	    }
	    else if (thrust[i]>0.79423) {
	    	voltage[i]=(thrust[i]+3.2786)/1.047;
	    }
	    else if (thrust[i]<-0.01095 AND thrust[i]>-1.179701){
	        voltage[i] = -0.0314 + sqrt(0.0314*0.0314-4*-0.0851*(0.0042-thrust[i]);
	    else if (thrust[i]<-1.179701) {
	    	voltage[i] = (thrust[i]-2.5582)/0.9609;
	    }
	}
	//Add saturation statement, ROS_INFO("VALUE TOO GREAT")
	
	//map voltages to motor commands
	//Conversion to integer between -128 and +127
	//linear for now TODO change mapping according to test
	for (int i=0; n<6; n++)
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

int main(int argc, char **argv)
{
	// ROS subscriber setup
	ros::init(argc,argv,"thrust_mapper");
	ros::NodeHandle n;
	ros::Subscriber thrust_subscriber = n.subscribe("/controls/wrench", 1000, thrust_callback);
	//add clock subscription

	//ROS Publisher setup
	ros::Publisher voltage_publisher = n.advertise<geometry_msgs::Custom>("/gazebo/voltageMsg", 100); //TODO change message type and name
	geometry_msgs::Twist twistMsg; //define variable

	bool ready = 0;
	ROS_INFO("thrust_mapper waiting for all subscribers to have content...");
	while (ready == 0)
	{
		ROS_DEBUG_THROTTLE(2,"Waiting...");
		ready = 1;		 
		if (thrust_subscriber.getNumPublishers() == 0) {ready = 0;}
		else {ROS_DEBUG_THROTTLE(2,"got thrust");}
	}
	ROS_INFO("All Subscribers Live. Starting thrust_mapper node!");
	ros::spin()
	return 0;
}