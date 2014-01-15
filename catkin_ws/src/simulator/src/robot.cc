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

namespace gazebo
{

/**
 * @brief class to manipulate robot behaviour
 * @author Dwijesh Bhageerutty
 * @author Jonathan Fokkan
 */
class Robot : public ModelPlugin
{
public:
	/**
	 * Constructor
	 */
	Robot() {
		int argc = 0;
		iterCount = 0;
		ros::init(argc, NULL, "Robot Plugin");
		std::cout<<"Robot plugin node Created"<<std::endl;
	};

	/**
	 * Destructor
	 */
	~Robot() {
		delete this->node;
	};

	/**
	 * Overidden.
	 */
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
		// Store the pointer to the model
		this->model = _parent;

		// private ROS NodeHandle
		this->node = new ros::NodeHandle("~");

		// ROS Subscriber
		// NOTE: should the queue be increased?
		this->twistSub = this->node->subscribe("simulator/robot_twist", 1, &Robot::moveCallback, this);

		// Thruster Forces
		this->thrusterForcesSub = this->node->subscribe("simulator/thruster_forces", 1000, &Robot::thrusterForcesCallBack, this);

		// /controls/wrench/
		this->controlsWrenchSub = this->node->subscribe("/controls/wrench", 1000, &Robot::controlsWrenchCallBack, this);

		// Listen to the update event. This event is broadcast every
		// simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robot::OnUpdate, this, _1));
	};

	/**
	 * Called on every simulation iteration
	 */
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
		ros::spinOnce();
	};

	/**
	 * When a Twist message is passed to the gazebo/twist topic,
	 * the linear and angular velocities for the robot are changed.
	 * @param msg Twist message
	 */
	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
		std::cout << "DEBUG::";
		std::cout << "linear vel: <" << msg->linear.x << ", " << msg->linear.y
				 << ", " << msg->linear.z << std::endl;
		std::cout << "angular vel: " << msg->angular.x << ", " << msg->angular.y
				 << ", " << msg->angular.z << std::endl;
		this->model->SetLinearVel(math::Vector3(msg->linear.x, msg->linear.y, -msg->linear.z));
		this->model->SetAngularVel(math::Vector3(msg->angular.x, msg->angular.y, msg->angular.z));
	};

	/**
	 * Function called when a message is passed to ThrusterForces topic.
	 * @param msg message of type ThrusterForces
	 */
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
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = wrench;
		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		std::cout << "fx:" << fx << ", fy:" << fy << " fz:" << fz; 
		std::cout << " taoX:" << taoX << ", taoY:" << taoY << " taoZ:" << taoZ << std::endl;

		client.call(applyBodyWrench);

		if (applyBodyWrench.response.success) {
			ROS_INFO("ApplyBodyWrench call successful.");
		} else {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
		applyDrag();
	};

	/**
	 * Calculates the drag vector based on the robot's velocity and
	 * applies the corresponding wrench.
	 */
	void applyDrag() {
		math::Vector3 linearVelocity = model->GetRelativeLinearVel();
		math::Vector3 angularVelocity = model->GetRelativeAngularVel();
		
		float u, v, w, p, q, r, magnitude, translationalDragMagnitude, taoX, taoY, taoZ;
		math::Vector3 translationalDragVector;

		u = linearVelocity.x;
		v = linearVelocity.y;
		w = linearVelocity.z;

		p = angularVelocity.x;
		q = angularVelocity.y;
		r = angularVelocity.z;
		
		magnitude = sqrt(u*u + v*v + w*w);
		
		if (magnitude > 1E-5) {
			translationalDragVector.x = u/magnitude;
			translationalDragVector.y = v/magnitude;
			translationalDragVector.z = w/magnitude;
		}
		
		translationalDragMagnitude = -.5 * .118 * 1000 * (u*u + v*v + w*w) * .8;
		translationalDragVector.x = translationalDragVector.x * translationalDragMagnitude;
		translationalDragVector.y = translationalDragVector.y * translationalDragMagnitude;
		translationalDragVector.z = translationalDragVector.z * translationalDragMagnitude;
		
		// wrench message
		geometry_msgs::Vector3 forceVector;
		forceVector.x = translationalDragVector.x;
		forceVector.y = translationalDragVector.y;
		forceVector.z = translationalDragVector.z;
				
		geometry_msgs::Vector3 torqueVector;
		torqueVector.x = -(KP * p * abs(p));
		torqueVector.y = -(KQ * q * abs(q));
		torqueVector.z = -(KR * r * abs(r));

		geometry_msgs::Wrench wrench;

		wrench.force = forceVector;
		wrench.torque = torqueVector;

		// ApplyBodyWrench message
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = wrench;

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		//std::cout << "Applying drag force:" << std::endl;
		//std::cout << "fx:" << forceVector.x << ", fy:" << forceVector.y << " fz:" << forceVector.z;
		//std::cout << " taoX:" << torqueVector.x << ", taoY:" << torqueVector.y << " taoZ:" << torqueVector.z << std::endl;
		
		client.call(applyBodyWrench);

		if (applyBodyWrench.response.success) {
			//ROS_INFO("ApplyBodyWrench call successful.");
		} else {
			//ROS_ERROR("ApplyBodyWrench call failed.");
		}
	}

	/**
	 * Function to handle Wrench messages passed to topic 'controls/wrench/'
	 * @param msg Wrench to be applied to robot
	 */
	void controlsWrenchCallBack(const geometry_msgs::Wrench msg) {
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = msg;

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		client.call(applyBodyWrench);

		if (applyBodyWrench.response.success) {
			ROS_INFO("ApplyBodyWrench call successful.");
		} else {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
		applyDrag();
	}

private:
	// CONSTANTS

	// for wrench computation
	const static float RX1 = .3; 
	const static float RX2 = -.3;
	const static float RY1 = .3;
	const static float RY2 = -.3;
	const static float RZ1 = .3;
	const static float RZ2 = -.3;

	// for drag computation
	const static float KP = 1;
	const static float KQ = 1;
	const static float KR = 1;

	/** Pointer to the model */
	physics::ModelPtr model;

	/** Pointer to the update event connection */
	event::ConnectionPtr updateConnection;

	/** ROS NodeHandle */
	ros::NodeHandle* node;

	/** robot_twist Subscriber */
	ros::Subscriber twistSub;
	
	/** ThrusterForces Subscriber */
	ros::Subscriber thrusterForcesSub;
	
	/** Controls wrench topic subscriber */
	ros::Subscriber controlsWrenchSub;
	
	int iterCount;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Robot)
}
