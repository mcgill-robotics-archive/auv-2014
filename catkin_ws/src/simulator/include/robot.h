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

class Robot : public ModelPlugin {

		public:
		Robot();
		~Robot();
		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
		void OnUpdate(const common::UpdateInfo & /*_info*/);
		void moveCallback(const geometry_msgs::Twist::ConstPtr& msg);
		void thrusterForcesCallBack(const simulator::ThrusterForces::ConstPtr& msg);
		void applyDrag();
		void controlsWrenchCallBack(const geometry_msgs::Wrench msg);
		
		private:
		const static float RX1 = .3; 
		const static float RX2 = -.3;
		const static float RY1 = .3;
		const static float RY2 = -.3;
		const static float RZ1 = .3;
		const static float RZ2 = -.3;

		const static float KP = 1;
		const static float KQ = 1;
		const static float KR = 1;

		physics::ModelPtr model;

		event::ConnectionPtr updateConnection;

		ros::NodeHandle* node;

		ros::Subscriber twistSub;
	
		ros::Subscriber thrusterForcesSub;
	
		ros::Subscriber controlsWrenchSub;
	
		int iterCount;
};

#endif
