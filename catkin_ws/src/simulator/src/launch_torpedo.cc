#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <stdio.h>
#include <time.h>
#include <stdlib.h>
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ApplyBodyWrench.h" 
#include "ros/ros.h"

namespace gazebo
{
    /**
     *@brief class to give the torpedo a vector
     *@author Jonathan Fokkan
     */ 
    class LaunchTorpedo : public ModelPlugin {
    public:

	/** 
	 * Load World
	 */
	void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) {
	    launch = true;

	    // Store the pointer to the model
	    this->model = _parent;

		// private ROS NodeHandle
		this->node = new ros::NodeHandle("~");

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&LaunchTorpedo::OnUpdate, this, _1));
	};

	/**
	 * Start Launch
	 */
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
	    if (launch) { 
			// apply the wrench
			gazebo_msgs::ApplyBodyWrench applyBodyWrench;
			applyBodyWrench.request.body_name = (std::string) "torpedo::body";
			
			geometry_msgs::Vector3 forceVector;
			forceVector.x = 3; 
			forceVector.y = 0; 
			forceVector.z = 0;
			
			geometry_msgs::Vector3 torqueVector;
			torqueVector.x = 0; 
			torqueVector.y = 0; 
			torqueVector.z = 0;
			
			applyBodyWrench.request.wrench.force = forceVector;
			applyBodyWrench.request.wrench.torque = torqueVector;
			
			applyBodyWrench.request.reference_frame = "torpedo::torpedo_reference_frame";
			//applyBodyWrench.request.start_time not specified -> it will start ASAP.
			applyBodyWrench.request.duration = ros::Duration(1);
			ros::ServiceClient client = node->serviceClient<gazebo_msgs::ApplyBodyWrench>("/gazebo/apply_body_wrench");
			client.call(applyBodyWrench);
		
			if (!applyBodyWrench.response.success) {
				ROS_ERROR("ApplyBodyWrench call failed.");
			}
			
			launch = false;
	    }
	};

    private:
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		bool launch;
		/** ROS NodeHandle */
		ros::NodeHandle* node;
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(LaunchTorpedo)
}
