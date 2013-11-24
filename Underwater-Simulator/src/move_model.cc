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
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
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
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
	    if(iteration < 1000) {
		// Apply a small linear velocity to the model.
		if(!appliedRight) {
		    this->model->SetLinearVel(math::Vector3(
						  .03,
						  (rand() % 2) * 0.001,
						  (rand() % 2) * 0.001));
		    appliedRight = true;
		    appliedLeft = false;
		}
		if ((rand() % 2) == 0){
		    ++iteration;
		}
	    } else {
		if(iteration > 2000) {
		    iteration = 0;
		}
		if(!appliedLeft) {
		    this->model->SetLinearVel(math::Vector3(
						  -.03,
						  (rand() % 2) * -0.001,
						  (rand() % 2) * -0.001));
		    appliedLeft = true;
		    appliedRight = false;
		}
		if ((rand() % 2) == 0){
		    ++iteration;
		}
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
