#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include <cstdlib> // Needed for rand()
#include <ctime> // Needed to seed random number generator with a time value

class RobotMove {
public:
    RobotMove(ros::NodeHandle& nh) {
	// Advertise a new publisher for the simulated robot's velocity command topic
	// (the second argument indicates that if multiple command messages are in
	// the queue to be sent, only the last command will be sent)
	commandPub = nh.advertise<geometry_msgs::Twist>("gazebo/robot_twist", 1);
    };

    // Send a velocity command
    void move(double x, double y, double z) {
	//std::cout<<"sending twist message of linear: "<<linearVelMPS<<" angular: "<<angularVelRadPS<<std::endl;
	geometry_msgs::Twist msg; // The default constructor will set all commands to 0
	msg.linear.x = x;
	msg.linear.y = y;
	msg.linear.z = z;
	//msg.angular.z = angularVelRadPS;
	commandPub.publish(msg);
    };

    void spin() {
	ros::Rate rate(10); // Specify the FSM loop rate in Hz


	while (ros::ok()) { // Keep spinning loop until user presses Ctrl+C
	    double x, y, z;

	    std::cout<<"x: ";
	    std::cin>>x;
	    std::cout<<std::endl;
	    std::cout<<"y: ";
	    std::cin>>y;
	    std::cout<<std::endl;
	    std::cout<<"z: ";
	    std::cin>>z;

	    std::cout<<"values "<<x<<y<<z<<std::endl;

	    move(x,y,z);

	    /*-----------------------------------------------------------------------*/
	    ros::spinOnce(); // Need to call this function often to allow ROS to
			     //process incoming messages
	    rate.sleep(); // Sleep for the rest of the cycle, to enforce the FSM loop rate
	}
    };


    const static double FORWARD_SPEED_MPS = 1.0;
    const static double ROTATE_SPEED_RADPS = M_PI/2;

protected:
    ros::Publisher commandPub; // Publisher to the simulated robot's velocity command topic
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "robot_move_test");// Initiate new ROS node
    ros::NodeHandle n;
    RobotMove mover(n);
    mover.spin(); // Execute loop

    return 0;
};
