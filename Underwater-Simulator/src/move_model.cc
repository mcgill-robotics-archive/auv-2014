#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

namespace gazebo
{
    class MoveModel : public ModelPlugin
    {
    public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	    {
		appliedRight = false;
		appliedLeft = false;
		iteration = 0;

		srand(time(NULL));
		// Store the pointer to the model
		this->model = _parent;

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		    boost::bind(&MoveModel::OnUpdate, this, _1));
	    };

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/)
	    {
		if(iteration < 1000) {
		    // Apply a small linear velocity to the model.
		    if(!appliedRight) {
			std::cout<<"Right"<<std::endl;
			this->model->SetLinearVel(math::Vector3(
						      .03,
						      (rand() % 5) * 0.01,
						      (rand() % 5) * 0.01));
			appliedRight = true;
			appliedLeft = false;
		    }
		    ++iteration;
		} else {
		    if(iteration > 2000) {
			iteration = 0;
		    }
		    if(!appliedLeft) {
			std::cout<<"Left"<<std::endl;
			this->model->SetLinearVel(math::Vector3(
						      -.03,
						      (rand() % 5) * -0.01,
						      (rand() % 5) * -0.01));
			appliedLeft = true;
			appliedRight = false;
		    }
		    ++iteration;
		}
	    };

    private:
	// Pointer to the model
	physics::ModelPtr model;
	// Pointer to the update event connection
	event::ConnectionPtr updateConnection;

	bool appliedRight;
	bool appliedLeft;
	int iteration;

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(MoveModel)
}
