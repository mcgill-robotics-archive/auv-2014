#include "ros/ros.h"
#include "simulator/ThrusterForces.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class ThrusterTest {
	public:
		ThrusterTest(ros::NodeHandle& nh) {
			commandPub = nh.advertise<simulator::ThrusterForces>("gazebo/simulator/thruster_forces", 1);
		};

		// Send a velocity command
		void thrust(double x1, double x2, double y1, double y2, double z1, double z2) {
			simulator::ThrusterForces msg; 
			msg.tx1 = x1;
			msg.tx2 = x2;
			msg.ty1 = y1;
			msg.ty2 = y2;
			msg.tz1 = z1;
			msg.tz2 = z2;
			commandPub.publish(msg);
		};

		void spin() {
			ros::Rate rate(10); 
			while (ros::ok()) { 
				double x1, x2, y1, y2, z1, z2;
				std::cout<<"x1: ";
				std::cin>>x1;
				std::cout<<std::endl;

				std::cout<<"x2: ";
				std::cin>>x2;
				std::cout<<std::endl;
			
				std::cout<<"y1: ";
				std::cin>>y1;
				std::cout<<std::endl;

				std::cout<<"y2: ";
				std::cin>>y2;
				std::cout<<std::endl;

				std::cout<<"z1: ";
				std::cin>>z1;
				std::cout<<std::endl;

				std::cout<<"z2: ";
				std::cin>>z2;
				std::cout<<std::endl;

				std::cout<<"values " << x1 << x2 << y1 << y2 << z1 << z2 << std::endl;

				thrust(x1, x2, y1, y2, z1, z2);

				ros::spinOnce(); 
				rate.sleep(); 
			}
		};

	protected:
		ros::Publisher commandPub; // Publisher to the simulated robot's thrust command topic
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "simulator_thruster_test");// Initiate new ROS node
    ros::NodeHandle n;
    ThrusterTest thruster(n);
    thruster.spin(); // Execute loop
    return 0;
};
