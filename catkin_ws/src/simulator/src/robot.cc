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
#include "std_msgs/Bool.h"

#define ABS_FLOAT(x) ((x < 0) ? -x : x)

namespace gazebo
{

/**
 * @brief class to manipulate robot behaviour
 * @author Dwijesh Bhageerutty
 */
class Robot : public ModelPlugin
{
public:

	/**
	 * Constructor
	 */
	Robot() {
		int argc = 0;
		ros::init(argc, NULL, "Robot Plugin");
		std::cout << "Robot plugin node created." << std::endl;
		noOfIterations = 0;
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
		if (_parent != NULL) this->model = _parent;

		// private ROS NodeHandle
		this->node = new ros::NodeHandle("~");

		// ROS Subscriber
		this->twistSub = this->node->subscribe("simulator/robot_twist", 1000, &Robot::moveCallback, this);

		// Thruster Forces
		this->thrusterForcesSub = this->node->subscribe("simulator/thruster_forces", 1000, &Robot::thrusterForcesCallBack, this);

		// /controls/wrench/
		this->controlsWrenchSub = this->node->subscribe("/controls/wrench", 1000, &Robot::controlsWrenchCallBack, this);

		// Marker Drop Sub topic subscriber 
		this->markerDropSub = this->node->subscribe("simulator/marker", 1000, &Robot::simulatorMarkerCallBack, this);

		// Listen to the update event. This event is broadcast every simulation iteration.
		this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&Robot::OnUpdate, this, _1));
	};

	/**
	 * Called on every simulation iteration
	 */
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
		++noOfIterations;

		if (noOfIterations % 500 == 0) {
			if (!applyDrag()) {
				ROS_ERROR("Drag application failed.");
			}
		}

		ros::spinOnce();
	};

	/**
	 * When a Twist message is passed to the gazebo/twist topic,
	 * the linear and angular velocities for the robot are changed.
	 * @param msg Twist message
	 */
	void moveCallback(const geometry_msgs::Twist::ConstPtr& msg) {
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
		applyBodyWrench.request.reference_frame = (std::string) "robot::dummy";
		applyBodyWrench.request.wrench = wrench;
		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		std::cout << "fx:" << fx << ", fy:" << fy << " fz:" << fz; 
		std::cout << " taoX:" << taoX << ", taoY:" << taoY << " taoZ:" << taoZ << std::endl;

		client.call(applyBodyWrench);

		if (!applyBodyWrench.response.success) {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
	};

	/**
	 * Calculates the drag vector based on the robot's velocity and
	 * applies the corresponding wrench.
	 */
	bool applyDrag() {
		geometry_msgs::Wrench wrench;

		wrench.force = calculateDragForce(model->GetRelativeLinearVel());
		wrench.torque = calculateDragTorque(model->GetRelativeAngularVel());;

		if (!shouldApplyForce(wrench.force.x, wrench.force.y, wrench.force.z, wrench.torque.x, wrench.torque.y, wrench.torque.z)) 
			return true;

		// ApplyBodyWrench message
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = wrench;

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);

		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");

		std::cout << "Applying drag force:";
		printWrenchMsg(wrench);
		client.call(applyBodyWrench);

		if (!applyBodyWrench.response.success) {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
		return applyBodyWrench.response.success;
	}
	
	/**
	 * Calculate current drag force
	 * @param linearVelocity robot's current linear velocity
	 */
	geometry_msgs::Vector3 calculateDragForce(math::Vector3 linearVelocity) {
		float magnitude, u, v, w, translationalDragMagnitude;
		geometry_msgs::Vector3 translationalDragVector;
		
		u = linearVelocity.x;
		v = linearVelocity.y;
		w = linearVelocity.z;
		
		magnitude = sqrt(u*u + v*v + w*w);
		
		if (magnitude > 1E-5) {
			translationalDragVector.x = u/magnitude;
			translationalDragVector.y = v/magnitude;
			translationalDragVector.z = w/magnitude;
		}
		
		//Drag force = -0.5 * Area * density * |speed|^2 * drag coefficient
		translationalDragMagnitude = -.5 * .118 * 1000 * (u*u + v*v + w*w) * .8;

		translationalDragVector.x = -(translationalDragVector.x * translationalDragMagnitude);
		translationalDragVector.y = -(translationalDragVector.y * translationalDragMagnitude);
		translationalDragVector.z = -(translationalDragVector.z * translationalDragMagnitude);

		return translationalDragVector;
	}

	/**
	 * Calculate current drag torque
	 * @param angularVelocity robot's current angular velocity
	 */
	geometry_msgs::Vector3 calculateDragTorque(math::Vector3 angularVelocity) {
		geometry_msgs::Vector3 torqueVector;
		torqueVector.x = -(KP * angularVelocity.x * ABS_FLOAT(angularVelocity.x));
		torqueVector.y = -(KQ * angularVelocity.y * ABS_FLOAT(angularVelocity.y));
		torqueVector.z = -(KR * angularVelocity.z * ABS_FLOAT(angularVelocity.z));
		return torqueVector;		
	}	
	
	/**
	 * Function to handle Wrench messages passed to topic 'controls/wrench/'
	 * @param msg Wrench to be applied to robot
	 */
	void controlsWrenchCallBack(const geometry_msgs::Wrench msg) {
		adjustModelYaw();

		// check if its worth trying to apply the wrench - e.g.: if everything is 0, just return.
		if (!shouldApplyForce(msg.force.x, msg.force.y, msg.force.z, msg.torque.x, msg.torque.y, msg.torque.z))	return;

		// apply the wrench
		gazebo_msgs::ApplyBodyWrench applyBodyWrench;
		applyBodyWrench.request.body_name = (std::string) "robot::body";
		applyBodyWrench.request.wrench = msg;
		applyBodyWrench.request.reference_frame = "robot::robot_reference_frame";

		//applyBodyWrench.request.start_time not specified -> it will start ASAP.
		applyBodyWrench.request.duration = ros::Duration(1);
		ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
		client.call(applyBodyWrench);
		
		if (!applyBodyWrench.response.success) {
			ROS_ERROR("ApplyBodyWrench call failed.");
		}
	}

	/**
	 * Adjust the model's yaw
	 */
	void adjustModelYaw() {
		float modelYaw = this->model->GetRelativePose().rot.GetYaw();
		math::Vector3 robotReferenceFrameAsEuler = this->model->GetLink("robot_reference_frame")->GetRelativePose().rot.GetAsEuler();
		float robotReferenceFrameYaw = robotReferenceFrameAsEuler.z;
		
		this->model->GetLink("robot_reference_frame")->SetRelativePose(
			math::Pose(this->model->GetLink("robot_reference_frame")->GetRelativePose().pos,
				math::Vector3(
					robotReferenceFrameAsEuler.x /*this->model->GetRelativePose().rot.GetRoll() + 3.14159265359*/, // same 
						robotReferenceFrameAsEuler.y /*this->model->GetRelativePose().rot.GetPitch()*/, // same
							this->model->GetRelativePose().rot.GetYaw() - 1.57079632679)), // model's yaw - pi/2 
								true,
									true);
	}

	void simulatorMarkerCallBack(const std_msgs::Bool::ConstPtr& msg) {
		if (msg->data) {
			ROS_INFO("Dropped Marker");
		}
	}

	bool shouldApplyForce(float u, float v, float w, float p, float q, float r) {
		return (inRangeForce(u) || inRangeForce(v) || inRangeForce(w) || inRangeForce(p) || inRangeForce(q) || inRangeForce(r));
	}	
	
	bool inRangeForce(float x) {
		if (x==0 || x==-0) return false;
		if (x>0) {
			if (x>.00001) return true;
		}
		if (x<-.00001) return true;
		return false;
	}

private:
	void printWrenchMsg(geometry_msgs::Wrench msg) {
		std::cout << "Applying wrench obtained from controls/wrench topic:" << std::endl;
		std::cout << "fx:" << msg.force.x << ", fy:" << msg.force.y << " fz:" << msg.force.z;
		std::cout << " taoX:" << msg.torque.x << ", taoY:" << msg.torque.y << " taoZ:" << msg.torque.z << std::endl;
	}

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

	/** Marker Drop Sub topic subscriber */
	ros::Subscriber markerDropSub;

	/** Torpedo Launch Sub topic subscriber */
	ros::Subscriber	torpedoLaunchSub;

	/** counter **/	
	int noOfIterations;
};

// Register this plugin with the simulator
GZ_REGISTER_MODEL_PLUGIN(Robot)
}
