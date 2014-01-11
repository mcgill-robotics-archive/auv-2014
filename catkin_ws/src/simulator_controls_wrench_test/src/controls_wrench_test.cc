#include "ros/ros.h"
#include "simulator/ThrusterForces.h"
#include <cstdlib> 
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"

class ControlsWrenchTest {
public:
	ControlsWrenchTest(ros::NodeHandle& nh) {
		commandPub = nh.advertise<geometry_msgs::Wrench>("/controls/wrench/", 1);
	};
	
	void spin() {
		while (ros::ok()) { 
			double fx, fy, fz, tx, ty, tz;
			std::cout<<"fx: ";
			std::cin>>fx;
			std::cout<<std::endl;

			std::cout<<"fy: ";
			std::cin>>fy;
			std::cout<<std::endl;
			
			std::cout<<"fz: ";
			std::cin>>fz;
			std::cout<<std::endl;

			std::cout<<"tx: ";
			std::cin>>tx;
			std::cout<<std::endl;

			std::cout<<"ty: ";
			std::cin>>ty;
			std::cout<<std::endl;

			std::cout<<"tz: ";
			std::cin>>tz;
			std::cout<<std::endl;

			std::cout<<"Applying wrench." << std::endl;

			geometry_msgs::Vector3 forceVector;
			forceVector.x = fx;
			forceVector.y = fy;
			forceVector.z = fz;
				
			geometry_msgs::Vector3 torqueVector;
			torqueVector.x = tx;
			torqueVector.y = ty;
			torqueVector.z = tz;

			geometry_msgs::Wrench wrench;

			wrench.force = forceVector;
			wrench.torque = torqueVector;
			
			commandPub.publish(wrench);			
			
			ros::spinOnce(); 
		}
	};

protected:
	ros::Publisher commandPub; // Publisher to the simulated robot's thrust command topic
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulator_controls_wrench_test");// Initiate new ROS node
    ros::NodeHandle n;
    ControlsWrenchTest controlsWrenchTest(n);
    controlsWrenchTest.spin(); // Execute loop
    return 0;
};
