#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "simulator/ThrusterForces.h"
#include "gazebo_msgs/ApplyBodyWrench.h" 
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include <unistd.h>
#include "std_msgs/Bool.h"

#define ABS_FLOAT(x) ((x < 0) ? -x : x)

namespace gazebo
{

/**
 * @brief class to manipulate robot behaviour
 * @author Dwijesh Bhageerutty
 */
class Robot : public ModelPlugin
{
public:
	
	Robot();
	~Robot();
	
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
	
	void OnUpdate(const common::UpdateInfo & /*_info*/);
	
	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
	
	void thrusterForcesCallBack(const simulator::ThrusterForces::ConstPtr& msg);
	
	bool applyDrag();	
	
	geometry_msgs::Vector3 calculateDragForce(math::Vector3 linearVelocity);
	
	geometry_msgs::Vector3 calculateDragTorque(math::Vector3 angularVelocity);	

	void controlsWrenchCallBack(const geometry_msgs::Wrench msg);

	void adjustModelYaw();	
	
	void simulatorMarkerCallBack(const std_msgs::Bool::ConstPtr& msg);

	bool shouldApplyForce(float u, float v, float w, float p, float q, float r);	

	bool inRangeForce(float x);

private:

	void printWrenchMsg(geometry_msgs::Wrench msg);

	// CONSTANTS

	// for wrench computation
	const static float RX1 = .3; 
	const static float RX2 = -.3;
	const static float RY1 = .3;
	const static float RY2 = -.3;
	const static float RZ1 = .3;
	const static float RZ2 = -.3;

	// for drag computation
	const static float KP = 1;
	const static float KQ = 1;
	const static float KR = 1;

	/** Pointer to the model */
	physics::ModelPtr model;

	/** Pointer to the update event connection */
	event::ConnectionPtr updateConnection;

	/** ROS NodeHandle */
	ros::NodeHandle* node;

	/** robot_twist Subscriber */
	ros::Subscriber twistSub;
	
	/** ThrusterForces Subscriber */
	ros::Subscriber thrusterForcesSub;
	
	/** Controls wrench topic subscriber */
	ros::Subscriber controlsWrenchSub;

	/** Marker Drop Sub topic subscriber */
	ros::Subscriber markerDropSub;

	/** Torpedo Launch Sub topic subscriber */
	ros::Subscriber	torpedoLaunchSub;

	/** counter **/	
	int noOfIterations;
};
}
