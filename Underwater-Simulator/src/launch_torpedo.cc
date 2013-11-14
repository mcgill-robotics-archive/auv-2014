#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

namespace gazebo
{
    class LaunchTorpedo : public ModelPlugin
    {
    public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
	    {
	    srand(time(NULL));
	    appliedRight = false;
		appliedLeft = false;
		iteration = 0;

		// Store the pointer to the model
		this->model = _parent;

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		    boost::bind(&LaunchTorpedo::OnUpdate, this, _1));
	    };

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/)
	    {
		this->model->SetLinearVel(math::Vector3(
					      .03,
					      (rand() % 5) * 0.01,
					      (rand() % 5) * 0.01));
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
    GZ_REGISTER_MODEL_PLUGIN(LaunchTorpedo)
}
