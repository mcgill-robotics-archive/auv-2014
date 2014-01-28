#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "ros/ros.h"

#ifndef LaunchTorpedo
#define LaunchTorpedo

class LaunchTorpedo : public ModelPlugin {

		public:

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
		void OnUpdate(const common::UpdateInfo & /*_info*/);
		enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STOP};
		const static double SPEED = 0.1;
		const static double PI = 3.14159265359;

		private:
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		bool launch;
		int iteration;
		enum FSM fsm;
		ros::Time rotateStartTime; // Start time of the rotation
		ros::Duration rotateDuration; // Duration of the rotation
};

#endif
