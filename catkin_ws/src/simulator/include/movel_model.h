#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#ifndef move_model
#define move_model

class MoveModel : public ModelPlugin {
		public:

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/);
		void OnUpdate(const common::UpdateInfo & /*_info*/);

		private:

		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		bool appliedRight;
		bool appliedLeft;
		int iteration;
};

#endif
