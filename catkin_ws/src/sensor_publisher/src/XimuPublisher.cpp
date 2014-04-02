#include <boost/asio.hpp> 
#include <boost/asio/serial_port.hpp>

#include "XimuReceiver.h"

#include "ros/ros.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include <cmath>

using namespace boost;

static unsigned int sequence = 0;
ros::Publisher pub, pub2;
geometry_msgs::Pose pos;
geometry_msgs::PoseStamped posStamped;
double prerotation[] = {1.0/sqrt(2.0), 1.0/sqrt(2.0),0.0,0.0};
double* q = new double[4];
asio::io_service serial_io;
asio::serial_port port(serial_io);
XimuReceiver receiver;

void multiplyQuaternions(double q[],double p[])
{
    double temp[4];
	temp[0] = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3];
    temp[1] = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2];
    temp[2] = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1];
    temp[3] = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
    q[0] = temp[0];
    q[1] = temp[1];
    q[2] = temp[2];
    q[3] = temp[3];
}


void spin() {
	port.open("/dev/ttyUSB0");
	port.set_option(asio::serial_port_base::baud_rate(115200));

	char c;

	while (true) {
		// Read 1 character into c, this will block
		// forever if no character arrives.
		asio::read(port, asio::buffer(&c,1));
		receiver.processNewChar(c);

if (receiver.isInertialAndMagGetReady()) {
                        InertialAndMagStruct ims = receiver.getInertialAndMag();
                        geometry_msgs::Vector3 acc = geometry_msgs::Vector3();
                        acc.x = ims.accX;
                        acc.y = ims.accY;
                        acc.z = ims.accZ;
                        pub2.publish(acc);
                }		

if (receiver.isQuaternionGetReady()) {
			QuaternionStruct quaternionStruct = receiver.getQuaternion();
			
 			q[0] = quaternionStruct.w;
			q[1] = quaternionStruct.x;
			q[2] = quaternionStruct.y;
			q[3] = quaternionStruct.z;
    	    multiplyQuaternions(q, prerotation);
			pos.orientation.w = q[0];
			pos.orientation.x = q[1];
			pos.orientation.y = q[2];
			pos.orientation.z = q[3];
			
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
printf("Starting XimuPublisher\n");
	ros::init(argc, argv, "x_imu_pose");

	pub = node.advertise<geometry_msgs::PoseStamped>("pose", 100);
        pub2 = node.advertise<geometry_msgs::Vector3>("acc", 100);
	
	spin();
	
	return 0;
}
