#include "gazebo/physics/physics.hh"
#include "gazebo/common/common.hh"
#include "gazebo/gazebo.hh"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace gazebo {
    /**
     * @brief class to create torpedo in Gazebo
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
	 * TODO
	 */

	void Load(physics::WorldPtr _parent, sdf::ElementPtr /*_sdf*/) {
	    this->world = _parent;

	    // private ROS Nodehandle
	    this->node = new ros::NodeHandle("~");

	    // ROS Subscriber
	    // NOTE: should the queue be increased?
	    this->sub = this->node->subscribe("launch", 1, &CreateTorpedo::createCallback, this);

	    // Listen to the update event. This event is broadcast every
	    // simulation iteration.
	    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
		boost::bind(&CreateTorpedo::OnUpdate, this, _1));
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
		sdf::SDF torpedoSDF;
		std::string torpedoString;
		std::string firstPart = "<sdf version ='1.4'>\
<model name='torpedo'>\
      <pose>";
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
      </link>\
      <plugin name='launch_torpedo' filename='liblaunch_torpedo.so'></plugin>\
    </model>\
</sdf>";

		physics::ModelPtr robot = world->GetModel("robot");
		math::Pose pose = robot->GetWorldPose();
		int x = pose.pos.x;
		int y = pose.pos.y;
		int z = pose.pos.z;

		double roll = pose.rot.GetRoll();
		double pitch = pose.rot.GetPitch();
		double yaw = pose.rot.GetYaw();

		torpedoString = firstPart + boost::lexical_cast<std::string>(x) + " " +
		    boost::lexical_cast<std::string>(y) + " " +
		    boost::lexical_cast<std::string>(z) + " " +
		    boost::lexical_cast<std::string>(roll) + " " +
		    boost::lexical_cast<std::string>(pitch) + " " +
		    boost::lexical_cast<std::string>(yaw) + secondPart;

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
