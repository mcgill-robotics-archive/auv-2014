#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Wrench.h"
#include "gazebo_msgs/ApplyBodyWrench.h" 

namespace gazebo {
    /**
     * @brief class to create torpedo in Gazebo
     * @author Dwijesh Bhageerutty
     * @author Jonathan Fokkan
     */
    class CreateTorpedo : public WorldPlugin {
    public:

	/**
	 * Constructor
	 */
	CreateTorpedo() {
	    int argc = 0;
	    ros::init(argc, NULL, "Create Torpedo Plugin");
	    std::cout<<"Create Torpedo plugin node Created"<<std::endl;
	}
	
	/**
	 * TODO: write doc
	 */
	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
	    this->world = _parent;

	    // private ROS Nodehandle
	    this->node = new ros::NodeHandle("~");

	    // ROS Subscriber
	    // NOTE: should the queue be increased?
	    this->sub = this->node->subscribe("simulator/torpedo", 1000, &CreateTorpedo::createCallback, this);

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(boost::bind(&CreateTorpedo::OnUpdate, this, _1));
	};

	// Called by the world update start event
	void OnUpdate(const common::UpdateInfo & /*_info*/) {
	    ros::spinOnce();
	};

	/**
	 * Give a torpedo a position and an orientation
	 */

	void createCallback(const std_msgs::Bool::ConstPtr& msg) {
	    
	    if (msg->data) {
			    
			// delete already existing torpedo
			if (world->GetModel("torpedo") != NULL){
				physics::ModelPtr robot = world->GetModel("robot");
				
				math::Pose pose = robot->GetWorldPose();
				world->GetModel("torpedo")->SetWorldPose(pose, true, true);
				
				double x,y,z,roll,yaw,pitch;
				// reference frame part
				math::Vector3 rrfRotAsEuler = robot->GetLink("robot_reference_frame")->GetRelativePose().rot.GetAsEuler();
				math::Vector3 rrfPosAsEuler = robot->GetLink("robot_reference_frame")->GetRelativePose().pos;
				math::Pose refPose;
				refPose.pos = rrfPosAsEuler;
				refPose.rot = rrfRotAsEuler;
				world->GetModel("torpedo")->GetLink("torpedo_reference_frame")->SetRelativePose(refPose, true, true);
				
				return;
			}    	
	    	
			sdf::SDF torpedoSDF;
			std::string torpedoString;
			std::string firstPart = "<sdf version ='1.4'><model name='torpedo'><pose>";
			std::string secondPart = "</pose>\
			  <static>false</static>\
			  <link name='body'>\
				<collision name='visual'>\
				  <geometry>\
				    <mesh>\
				  <uri>file://../models/Torpedo.dae</uri>\
				  <scale> .01 .01 .01</scale>\
				</mesh>\
				  </geometry>\
				</collision>\
				<visual name='visual'>\
				  <geometry>\
				    <mesh>\
				  <uri>file://../models/Torpedo.dae</uri>\
				  <scale> .01 .01 .01</scale>\
				</mesh>\
				  </geometry>\
				</visual>\
			  </link>";
			  
		  std::string thirdPart = "<plugin name='launch_torpedo' filename='liblaunch_torpedo.so'></plugin></model></sdf>";
			 
			physics::ModelPtr robot = world->GetModel("robot");
			double x,y,z,roll,yaw,pitch;

			// reference frame part
			math::Vector3 rrfRotAsEuler = robot->GetLink("robot_reference_frame")->GetRelativePose().rot.GetAsEuler();
			math::Vector3 rrfPosAsEuler = robot->GetLink("robot_reference_frame")->GetRelativePose().pos;

			x = rrfPosAsEuler.x;
			y = rrfPosAsEuler.y;
			z = rrfPosAsEuler.z;
		
			roll = rrfRotAsEuler.x;
			pitch = rrfRotAsEuler.y;
			yaw = rrfRotAsEuler.z;
		
		  std::string referenceFramePart = "<joint name='trf_joint' type='revolute'>\
			<parent>body</parent>\
			<child>torpedo_reference_frame</child>\
			<axis>0 0 1</axis>\
			</joint>\
			<link name='torpedo_reference_frame'>\
			<pose>" + boost::lexical_cast<std::string>(x) + " " +
				boost::lexical_cast<std::string>(y) + " " +
				boost::lexical_cast<std::string>(z) + " " +
				boost::lexical_cast<std::string>(roll) + " " +
				boost::lexical_cast<std::string>(pitch) + " " +
				boost::lexical_cast<std::string>(yaw) + "</pose>\
				</link>";
			// reference frame part done

			math::Pose pose = robot->GetWorldPose();
			x = pose.pos.x;
			y = pose.pos.y;
			z = pose.pos.z;

			roll = pose.rot.GetRoll();
			pitch = pose.rot.GetPitch();
			yaw = pose.rot.GetYaw();

			torpedoString = firstPart + boost::lexical_cast<std::string>(x) + " " +
				boost::lexical_cast<std::string>(y) + " " +
				boost::lexical_cast<std::string>(z) + " " +
				boost::lexical_cast<std::string>(roll) + " " +
				boost::lexical_cast<std::string>(pitch) + " " +
				boost::lexical_cast<std::string>(yaw) 
				+ secondPart 
				+ referenceFramePart 
				+ thirdPart;

			torpedoSDF.SetFromString(torpedoString);
			world->InsertModelSDF(torpedoSDF);
	    }
	};

    private:
		physics::WorldPtr world;
		event::ConnectionPtr updateConnection;
		// ROS NodeHandle
		ros::NodeHandle* node;
		//ROS Subscriber
		ros::Subscriber sub;
};

GZ_REGISTER_WORLD_PLUGIN(CreateTorpedo)
}
