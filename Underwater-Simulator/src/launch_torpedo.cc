#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"


namespace gazebo
{
    class LaunchTorpedo : public ModelPlugin {
    public:
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
	    launch = true;
	    iteration = 0;
	    fsm = FSM_MOVE_FORWARD;
	    srand(time(NULL));
	    // Store the pointer to the model
	    this->model = _parent;

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&LaunchTorpedo::OnUpdate, this, _1));
	};

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
	    /*
	      Note:
	      Linear velocity sends
	    */
	    if (launch) {
		math::Pose pose = this->model->GetWorldPose();

		int x = pose.pos.x;
		int y = pose.pos.y;
		int z = pose.pos.z;
		//std::cout<<"x :"<<x<<", y: "<<y<<", z: "<<z;//<<std::endl;

		double roll = pose.rot.GetRoll();
		double pitch = pose.rot.GetPitch();
		double yaw = pose.rot.GetYaw();
		//std::cout<<" | roll :"<<roll
		//<<", pitch: "<<pitch<<", yaw: "<<yaw<<std::endl;

		/* Note:
		   -at roll, pitch and yaw 0,0,0
		   set velocity straight along yaxis
		   - roll is actually pitch
		   - pitch is actuall roll
		   - yaw is yaw
		*/

		double xvel = 0.0, yvel = 0.1, zvel = 0.0;


		/*
		 */
		if(iteration < 4285) {
		    fsm = FSM_ROTATE;
		} else if (iteration == 4286){
		    fsm = FSM_STOP;
		}
		++iteration;

		if (abs(roll - PI) < 0.01) {
		    //std::cout<<"----->"<<iteration<<std::endl;
		    //std::cout<<fsm<<std::endl;
		}

		if (fsm == FSM_ROTATE) {
		    this->model->SetAngularVel(math::Vector3(0.5, 0, 0));
		} else if (fsm == FSM_MOVE_FORWARD) {
		    xvel = cos(yaw) * cos(roll) * SPEED;
		    yvel = sin(yaw) * cos(roll) * SPEED;
		    zvel = sin(roll) * SPEED;
		    //std::cout<<"x: "<<xvel<<" y: "<<" z: "<<zvel<<std::endl;
		    this->model->SetLinearVel(math::Vector3(
						  xvel,
						  yvel,
						  zvel));
		} else if (fsm == FSM_STOP) {
		    this->model->SetAngularVel(math::Vector3(0, 0, 0));
		    this->model->SetLinearVel(math::Vector3(0, 0, 0));
		    fsm = FSM_MOVE_FORWARD;
		}
		//this->model->SetLinearVel(math::Vector3(0, 0.1, 0));
	    }
	};

	// CONSTANTS
	// TODO adjust the speed
	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STOP};
	const static double SPEED = 0.1;
	const static double PI = 3.14159265359;


    private:
	// Pointer to the model
	physics::ModelPtr model;
	// Pointer to the update event connection
	event::ConnectionPtr updateConnection;

	bool launch;
	int iteration;
	enum FSM fsm;
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LaunchTorpedo)
}
