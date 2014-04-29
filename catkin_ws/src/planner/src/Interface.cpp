#include "Interface.h"

//totally going to need to tweak these values at some point
double maxDepthError = 0.5;
double maxHeadingError = 0.5;

ros::Subscriber estimatedState_subscriber;
ros::Subscriber estimatedDepth_subscriber;

ros::ServiceClient btClient;

ros::Publisher wrench_pub;
ros::Publisher CV_objs_pub;
ros::Publisher control_pub;
ros::Publisher checkpoints_pub;
ros::Publisher taskPubFront;
ros::Publisher taskPubDown;

geometry_msgs::PoseStamped myPose;
geometry_msgs::PoseStamped relativePose;
/**
 * Our current orientation from state estimation
 */
double visible_XPos;
double visible_YPos;
double visible_Depth;
double visible_Yaw;
double visible_Pitch;

/**
 * The duration of time the planner will wait for the TF broadcaster to setup before timing out. (in seconds I think)
 */
int const TF_BROADCASTER_TIMOUT_PERIOD = 10;

/**
 * Defines in which folder are stored the XML files.
 */
std::string xmlFilesPath;

/**
 * Object the we currently want CV to look for
 */
std::string visionObj;

/*
* Number of individual LEDs on our half of the blinky tape
*/
int blinkyLength = 30;

void spinThread() {
	ros::spin();
}

void estimatedDepth_callback(const std_msgs::Float64 msg) {
	visible_Depth = msg.data;
}

/**
 * determines the position and heading of our robot
 */
void setOurPose() {
	tf::TransformListener listener;
	geometry_msgs::PoseStamped emptyPose;
	emptyPose.header.frame_id = "/robot/rotation_center";
	emptyPose.pose.position.x = 0.0;
	emptyPose.pose.position.y = 0.0;
	emptyPose.pose.position.z = 0.0;
	emptyPose.pose.orientation.x = 0.0;
	emptyPose.pose.orientation.y = 0.0;
	emptyPose.pose.orientation.z = 0.0;
	emptyPose.pose.orientation.w = 1.0;
	listener.transformPose("/robot/rotation_center", emptyPose, myPose);
}

/**
 * gets the position/heading of robot relative to chosen object
 */
void setTransform(std::string referenceFrame) {
	tf::TransformListener listener;
	//setOurPose();
	geometry_msgs::PoseStamped emptyPose;
	emptyPose.header.frame_id = referenceFrame;
	emptyPose.pose.position.x = 0.0;
	emptyPose.pose.position.y = 0.0;
	emptyPose.pose.position.z = 0.0;
	emptyPose.pose.orientation.x = 0.0;
	emptyPose.pose.orientation.y = 0.0;
	emptyPose.pose.orientation.z = 0.0;
	emptyPose.pose.orientation.w = 1.0;
	try {
		listener.waitForTransform("/robot/rotation_center", emptyPose.header.frame_id,
				ros::Time(0), ros::Duration(0.4));
		listener.transformPose("/robot/rotation_center", emptyPose, relativePose);
	} catch (tf::TransformException ex) {
		ROS_ERROR("%s", ex.what());
	}
}

std::vector<double> getTransform() {
	std::vector<double> relativeDistance;
	relativeDistance.push_back(relativePose.pose.position.x);
	relativeDistance.push_back(relativePose.pose.position.y);
	relativeDistance.push_back(0.0);
	relativeDistance.push_back(0.0);
	relativeDistance.push_back(8.8);
	//relativeDistance.push_back();
	//relativeDistance.push_back();
	return relativeDistance;
}

bool areWeThereYet(std::vector<double> desired) {
	//if (estimatedDepth_subscriber.getNumPublishers() == 0) {return false;}
	//if (estimatedState_subscriber.getNumPublishers() == 0) {return false;}
	double xError = visible_XPos - desired.at(0);
	double yError = visible_YPos - desired.at(1);
	double pitchError = visible_Pitch - desired.at(2);
	double yawError = visible_Yaw - desired.at(3);
	double depthError = visible_Depth - desired.at(4);

	/******below will show the errors in case of issues (comment out if needed)
	 std::cout<<"xError: " << xError <<std::endl;
	 std::cout<<"yError: " << yError <<std::endl;
	 std::cout<< "pitchError: " << pitchError <<std::endl;
	 std::cout<<"yawError: " << yawError <<std::endl;
	 std::cout<<"depthError: " << depthError << "\n---------------------------------------\n\n"; */

	return (xError < .1 && yError < .1 && pitchError < .1 && yawError < .1
			&& depthError < .1);
}

//blame alan
bool areWeThereYet_tf(std::string referenceFrame, std::vector<double> desired) {
	//if (estimatedDepth_subscriber.getNumPublishers() == 0) {return false;}
	//if (estimatedState_subscriber.getNumPublishers() == 0) {return false;}
	setTransform(referenceFrame);
	//positional bounds
	bool xBounded = abs(relativePose.pose.position.x - desired.at(0)) < 1;
	ROS_INFO("Interface::x %f",relativePose.pose.position.x);
	bool yBounded = abs(relativePose.pose.position.y - desired.at(1)) < 1;
	ROS_INFO("Interface::y %f",relativePose.pose.position.y);
	bool zBounded = abs(relativePose.pose.position.z - desired.at(4)) < 2;
	//rotational bounds
	double x = relativePose.pose.orientation.x;
	double y = relativePose.pose.orientation.y;
	double z = relativePose.pose.orientation.z;
	double w = relativePose.pose.orientation.w;
	double pitch = 57.2957795130823f
			* -atan(
					(2.0f * (x * z + w * y))
							/ sqrt(
									1.0f
											- pow((2.0f * x * z + 2.0f * w * y),
													2.0f)));
	double yaw = 57.2957795130823f
			* atan2(2.0f * (x * y - w * z), 2.0f * w * w - 1.0f + 2.0f * x * x);
	bool pitchBounded = abs(pitch - desired.at(2)) < 5;
	bool yawBounded = abs(yaw - desired.at(3)) < 5;

	return (xBounded && yBounded);
}

void setVisionObj(int objIndex) {
	planner::CurrentCVTask msgFront;
	planner::CurrentCVTask msgDown;

	msgFront.currentCVTask = msgFront.NOTHING;
	msgDown.currentCVTask = msgDown.NOTHING;
	switch (objIndex) {
	case 0:
		msgFront.currentCVTask = msgFront.NOTHING;
		msgDown.currentCVTask = msgFront.NOTHING;
		break;
	case 1:
		msgFront.currentCVTask = msgFront.GATE;
		msgDown.currentCVTask = msgFront.NOTHING;
		break;
	case 2:
		msgDown.currentCVTask = msgFront.LANE;
		msgFront.currentCVTask = msgFront.NOTHING;
		break;
	case 3:
		msgFront.currentCVTask = msgFront.BUOY;
		msgDown.currentCVTask = msgFront.NOTHING;
		break;
	}

	taskPubFront.publish(msgFront);
	taskPubDown.publish(msgDown);
}

void weAreHere(std::string task) {
	std_msgs::String msg;
	msg.data = task;
	checkpoints_pub.publish(msg);
}

void setPoints(double pointControl[]) {
	planner::setPoints msgControl;

	msgControl.XPos.isActive = pointControl[0];
	msgControl.XPos.data = pointControl[1];

	msgControl.YPos.isActive = pointControl[2];
	msgControl.YPos.data = pointControl[3];

	msgControl.Yaw.isActive = pointControl[4];
	msgControl.Yaw.data = pointControl[5];

	msgControl.Pitch.isActive = pointControl[6];
	msgControl.Pitch.data = pointControl[7];

	msgControl.XSpeed.isActive = pointControl[8];
	msgControl.XSpeed.data = pointControl[9];

	msgControl.YSpeed.isActive = pointControl[10];
	msgControl.YSpeed.data = pointControl[11];

	msgControl.YawSpeed.isActive = pointControl[12];
	msgControl.YawSpeed.data = pointControl[13];

	msgControl.Depth.isActive = pointControl[14];
	msgControl.Depth.data = pointControl[15];

	msgControl.Frame = "/target/gate";

	control_pub.publish(msgControl);
}

void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth) {
	double pointControl[16] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, x_speed, 1, y_speed,
			1, yaw_speed, 1, depth };
	setPoints(pointControl);
}

void setPosition(std::vector<double> desired) {
	double pointControl[16] =
			{ 1, desired.at(0), 1, desired.at(1), 0, desired.at(2), 0,
					desired.at(3), 0, 0, 0, 0, 0, 0, 1, desired.at(4) };
	setPoints(pointControl);
}

void setRobotInitialPosition(ros::NodeHandle n, int x, int y, int z) {

  ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
  gazebo_msgs::SetModelState setmodelstate;
  gazebo_msgs::ModelState modelstate;

  geometry_msgs::Pose start_pose;
  start_pose.position.x = x;
  start_pose.position.y = y;
  start_pose.position.z = z;
  start_pose.orientation.x = 0.0;
  start_pose.orientation.y = 0.0;
  start_pose.orientation.z = 1.0;
  start_pose.orientation.w = 0.0;

  geometry_msgs::Twist start_twist;
  start_twist.linear.x = 0.0;
  start_twist.linear.y = 0.0;
  start_twist.linear.z = 0.0;
  start_twist.angular.x = 0.0;
  start_twist.angular.y = 0.0;
  start_twist.angular.z = 0.0;

 
  modelstate.model_name = (std::string) "robot";
  modelstate.reference_frame = (std::string) "world";
  modelstate.pose = start_pose;
  modelstate.twist = start_twist;

  setmodelstate.request.model_state = modelstate;
  //client.call(setmodelstate);

  ros::service::waitForService("/gazebo/set_model_state", -1);
  if(client.call(setmodelstate))
    { 
      ROS_INFO("Set robot's position: Success");
    }
    else
    {
      ROS_ERROR("Failed to call service ");
  }
}
void setRobotInitialPosition(ros::NodeHandle n, int task_id) {
	switch (task_id) {
		case (0) :
			setRobotInitialPosition(n, 2.7, -3.5, 1);
			break;
		case (1) : //TODO: find starting position for lane task
			setRobotInitialPosition(n, 2.7, -3.5, 1);
			break;
		case (2) : //TODO: find starting position for buoy task
			setRobotInitialPosition(n, 2.7, -3.5, 1);
			break;	
	}
}

blinky::RGB getColorValues(int myColor) {
	blinky::RGB color;
	switch (myColor) {
		case (0) :
			color.r = 255;
			color.g = 0;
			color.b = 0;
			break;
		case (1):
			color.r = 0;
			color.g = 255;
			color.b = 0;
			break;
		case (2):
			color.r = 0;
			color.g = 0;
			color.b = 255;
			break;
		case (3):
			color.r = 255;
			color.g = 255;
			color.b = 255;
			break;
		case (4):
			color.r = 0;
			color.g = 0;
			color.b = 0;
			break;
		case (5):
			color.r = 255;
			color.g = 0;
			color.b = 255;
			break;
	}
	return color;
}
//30 blinkies
void updateBlinkyTape(int myColor) {
	std::vector<blinky::RGB> colors;
	blinky::RGB color = getColorValues(myColor);

	for(int i = 0; i < blinkyLength; i++) {
		colors.push_back(color);
	}
	//send_colorList(colors);
	blinky::UpdatePlannerLights srv;
	srv.request.colors = colors;
	if(!btClient.call(srv)) {
		ROS_ERROR("failed to call service UpdatePlannerLights");
	}
}

int get_task_id(std::string name) {
	if (name == "gate") return 0;
	if (name == "lane") return 1;
	if (name == "buoy") return 2;
}

int main(int argc, char **argv) {
  std::string starting_task;
	ros::init(argc, argv, "Planner");
	ros::NodeHandle n;

	estimatedDepth_subscriber = n.subscribe("state_estimation/depth", 1000, estimatedDepth_callback);

	btClient = n.serviceClient<blinky::UpdatePlannerLights>("update_planner_lights");

	taskPubFront = n.advertise<planner::CurrentCVTask>("current_cv_task_front", 1000);
	taskPubDown = n.advertise<planner::CurrentCVTask>("current_cv_task_down", 1000);
	checkpoints_pub = n.advertise<std_msgs::String>("planner/task", 1000);
	control_pub = n.advertise<planner::setPoints>("setPoints", 1000);

	n.param<std::string>("Planner/xml_files_path", xmlFilesPath, "");
  n.param<std::string>("Planner/starting_task", starting_task, "gate"); //default ""?
  
  int start_task; int end_task;
  n.param<int>("Planner/start_task", start_task, 0);
	n.param<int>("Planner/end_task", end_task, 0);

std::cout<<starting_task<<std::endl;
	// Waits until the environment is properly setup until the planner actually starts.
	bool ready = 0;
	while (ready == 0) {
		ROS_DEBUG_THROTTLE(2,
				"Waiting for the environment to be setup properly before starting the planner...");

		if (estimatedDepth_subscriber.getNumPublishers() == 0) {
			// We need to wait for the publishers to be publishing their data.
			ready = 0;
		} else {
			ROS_DEBUG_THROTTLE(2, "Here ye Heare ye");
			ready = 1;
		}

		try {
			tf::TransformListener listener;
			geometry_msgs::PoseStamped emptyPose;
			emptyPose.header.frame_id = "/robot/rotation_center";
			emptyPose.pose.position.x = 0.0;
			emptyPose.pose.position.y = 0.0;
			emptyPose.pose.position.z = 0.0;
			emptyPose.pose.orientation.x = 0.0;
			emptyPose.pose.orientation.y = 0.0;
			emptyPose.pose.orientation.z = 0.0;
			emptyPose.pose.orientation.w = 1.0;

			// Waits until the TF broadcaster is ready and will timeout after the time specified in 'TF_BROADCASTER_TIMOUT_PERIOD'.
			listener.waitForTransform("/target/gate", emptyPose.header.frame_id,
					ros::Time(0), ros::Duration(TF_BROADCASTER_TIMOUT_PERIOD));
			listener.transformPose("/target/gate", emptyPose, relativePose);
		} catch (tf::TransformException ex) {
			ROS_INFO("Planner::Interface - Error thrown in TF listener.");
			ROS_ERROR("%s", ex.what());
		}
		ROS_INFO("Planner::Interface - waiting for dependencies");
	}

	ros::Rate loop_rate(10);
	/****This is ros::spin() on a seperate thread*****/
	boost::thread spin_thread(&spinThread);

	setRobotInitialPosition(n, get_task_id(starting_task));

	ROS_INFO("Planner::Interface - beginning routine");
	run_routine(start_task, end_task);

	return 0;
}
