#ifndef CreateTorpedo
#define CreateTorpedo

#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "std_msgs/Bool.h"


class CreateTorpedo : public WorldPlugin {

	public:

	CreateTorpedo();
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/);
	void OnUpdate(const common::UpdateInfo & /*_info*/);
	void createCallback(const std_msgs::Bool::ConstPtr& msg);

	private:

	physics::WorldPtr world;
	event::ConnectionPtr updateConnection;
	ros::NodeHandle* node;
	ros::Subscriber sub;

};

#endif
