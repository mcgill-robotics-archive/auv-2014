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
	    this->twistSub = this->node->subscribe("robot_twist", 1, &Robot::moveCallback, this);
	    
	    // Thruster Forces
		this->thrusterForcesSub = this->node->subscribe("thruster_forces", 1000, &Robot::thrusterForcesCallBack, this);

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robot::OnUpdate, this, _1));
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

	    this->model->SetLinearVel(math::Vector3(msg->linear.x, msg->linear.y, msg->linear.z));
	    this->model->SetAngularVel(math::Vector3(msg->angular.x, msg->angular.y, msg->angular.z));
	};

	// handle thruster forces being passed
	void thrusterForcesCallBack(const simulator::ThrusterForces::ConstPtr& msg){
		float fx, fy, fz, taoX, taoY, taoZ;
		
		// map individual thrusts to net wrench
		fx = msg->tx1 + msg->tx2;
		fy = msg->ty1 + msg->ty2;
		fz = msg->tz1 + msg->tz2;
		
		taoX = 0;
		taoY = RZ1 * msg->tz1 + RZ2 * msg->tz2;	
		taoZ = RX1 * msg->tx1 + RX2 * msg->tx2 + RY1 * msg->ty1 + RY2 * msg->ty2;
		
		// make service call
		//rosservice call /gazebo/apply_body_wrench '{body_name: "my_robot::body" , wrench: { force: { x: .01, y: .01, z: .01}, torque: { x: -0.1, y: 0 , z: 0 } }, start_time: 10000000000, duration: 1000000000 }'
		
		// wrench msg
		geometry_msgs::Vector3 forceVector;
		forceVector.x = fx; 
		forceVector.y = fy; 
		forceVector.z = fz;
		
		geometry_msgs::Vector3 torqueVector;
		torqueVector.x = taoX; 
		torqueVector.y = taoY; 
		torqueVector.z = taoZ;

		geometry_msgs::Wrench wrench;
		
		wrench.force = forceVector;
		wrench.torque = torqueVector;
		
		// ApplyBodyWrench msg
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "my_robot::body";
		applyBodyWrench.request.wrench = wrench;
		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1000);
		
		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
		
		std::cout << "fx:" << fx << ", fy:" << fy << " fz:" << fz; 
		std::cout << " taoX:" << taoX << ", taoY:" << taoY << " taoZ:" << taoZ << std::endl;
		
		client.call(applyBodyWrench);
		
		if (applyBodyWrench.response.success) {
			ROS_INFO("ApplyBodyWrench call successful.");
		} else {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
	};

	// CONSTANTS
	enum FSM {FSM_MOVE_FORWARD, FSM_ROTATE, FSM_STOP};
	const static double SPEED = 0.1;
	const static double PI = 3.14159265359;

	// for wrench computation
	const static float RX1 = .3; 
	const static float RX2 = -.3;
	const static float RY1 = .3;
	const static float RY2 = -.3;
	const static float RZ1 = .3;
	const static float RZ2 = -.3;

    private:
		// Pointer to the model
		physics::ModelPtr model;
		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;

		// ROS NodeHandle
		ros::NodeHandle* node;
		//ROS Subscriber
		ros::Subscriber twistSub;
		ros::Subscriber thrusterForcesSub;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(Robot)
}
