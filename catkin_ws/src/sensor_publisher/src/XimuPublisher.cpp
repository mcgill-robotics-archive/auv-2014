#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp>

#include "XimuReceiver.h"

#include "ros/ros.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"

using namespace boost;

static unsigned int sequence = 0;
ros::Publisher pub;
geometry_msgs::Pose pos;
geometry_msgs::PoseStamped posStamped;

asio::io_service io;
asio::serial_port port(io);
XimuReceiver receiver;

void spin() {
	port.open("/dev/ttyUSB0");
	port.set_option(asio::serial_port_base::baud_rate(115200));

	char c;

	while (true) {
		// Read 1 character into c, this will block
		// forever if no character arrives.
		asio::read(port, asio::buffer(&c,1));
		receiver.processNewChar(c);

		if (receiver.isQuaternionGetReady()) {
			QuaternionStruct quaternionStruct = receiver.getQuaternion();
			
			pos.orientation.w = quternionStruct.w;
			pos.orientation.x = quternionStruct.x;
			pos.orientation.y = quternionStruct.y;
			pos.orientation.z = quternionStruct.z;
			
			posStamped = geometry_msgs::PoseStamped();
			posStamped.pose = pos;
			posStamped.header.seq = sequence++;
			posStamped.header.stamp = ros::Time::now();
			posStamped.header.frame_id = "base_footprint";

			pub.publish(posStamped);
		}
	}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "x_imu_pose");
	ros::NodeHandle node;

	pub = node.advertise<geometry_msgs::PoseStamped>("pose_estimation", 100);
	
	spin();
	
	return 0;
}
