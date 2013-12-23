#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


namespace gazebo
{
    class Robot : public ModelPlugin
    {
    public:
	Robot() {
	    int argc = 0;
	    ros::init(argc, NULL, "Robot Plugin");
	    std::cout<<"Robot plugin node Created"<<std::endl;
	};

	~Robot() {
	    delete this->node;
	};

	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
	    // Store the pointer to the model
	    this->model = _parent;

	    // private ROS Nodehandle
	    this->node = new ros::NodeHandle("~");

	    // ROS Subscriber
	    // NOTE: should the queue be increased?
	    this->sub = this->node->subscribe("robot_twist", 1, &Robot::moveCallback, this);

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&Robot::OnUpdate, this, _1));
	};

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
	    //std::cout<<"spin"<<std::endl;
	    ros::spinOnce();
	};

	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
	    std::cout<<"DEBUG::";
	    std::cout<<"linear vel: <"<<msg->linear.x<<", "<<msg->linear.y
		     <<", "<<msg->linear.z<<std::endl;
	    std::cout<<"angular vel: "<<msg->angular.x<<", "<<msg->angular.y
		     <<", "<<msg->angular.z<<std::endl;

	    this->model->SetLinearVel(math::Vector3(msg->linear.x,
						     msg->linear.y,
						     msg->linear.z));
	    this->model->SetAngularVel(math::Vector3(msg->angular.x,
						     msg->angular.y,
						     msg->angular.z));
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

	// ROS NodeHandle
	ros::NodeHandle* node;
	//ROS Subscriber
	ros::Subscriber sub;

	/*
	bool launch;
	int iteration;
	enum FSM fsm;
	ros::Time rotateStartTime; // Start time of the rotation
	ros::Duration rotateDuration; // Duration of the rotation
	*/

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Robot)
}
