#include "Planner.h"
#include "Task.h"
//when you create a new task, include its header file here to prevent circular dependencies
#include "Task_Gate.h"
#include "Lost_Gate.h"
#include "Task_Kill.h"
#include "Task_Lane.h"

double ourDepth;
int inSim;
std::string starting_task;

void spinThread() {
	ros::spin();
}

void estimatedDepth_callback(const std_msgs::Float64 msg) {
	ourDepth = msg.data;
}

void Planner::seeObject_callback(const std_msgs::Bool msg) {
	seeObject = msg.data;
}

double Planner::getDepth() {
	return ourDepth;
}

bool Planner::getSeeObject() {
	return seeObject;
}

bool Planner::isOpenLoopDepth() {
	return openLoopDepth;
}

double Planner::getOpenLoopDepthSpeed() {
    return openLoopDepthSpeed;
}

double Planner::getCloseLoopDepth() {
    return closeLoopDepth;
}

double Planner::getSurgeSpeed() {
    return surgeSpeed;
}

double Planner::getYawTimeout() {
	return yawTimeout;
}

double Planner::getCloseLoopDepthTimeout() {
	return closeLoopDepthTimeout;
}

double Planner::getOpenLoopDepthTimeout() {
	return openLoopDepthTimeout;
}

double Planner::getGateTimeout() {
	return gateTimeout;
}

double Planner::getLaneTimeout() {
	return laneTimeout;
}

double Planner::getYawBound() {
	return yawBound;
}

double Planner::getXYYawFromIMU(std::string frame) {
	tf::StampedTransform transform = getStampedTransform(frame);

	currentX = transform.getOrigin().x();
	currentY = transform.getOrigin().y();
	//ignore depth. it comes in on a topic

	tf::Quaternion q = transform.getRotation(); //save the rotation as a quaternion
	tf::Matrix3x3 m(q); //convert quaternion to matrix

	double roll_unused; //unused, but needs to be sent to getRPY method
	double pitch_unused;

	m.getEulerYPR(currentYaw, pitch_unused, roll_unused);
}

double Planner::getCurrentX(std::string referenceFrame) {
	getXYYawFromIMU(referenceFrame);
	return currentX;
}

double Planner::getCurrentY(std::string referenceFrame) {
	getXYYawFromIMU(referenceFrame);
	return currentY;
}

double Planner::getCurrentYaw(std::string referenceFrame) {
	getXYYawFromIMU(referenceFrame);
	return currentYaw;
}

//##############################################################################
// Newly added parameters for the multiple competition scenario:         [START]
//##############################################################################
bool Planner::getUseCvForLane() {
	return useCvForLane;
}

bool Planner::getUseHardcodedLaneAngleAfterGate() {
	return useHardcodedLaneAngleAfterGate;
}

double Planner::getHardcodedRelativeLaneAngleAfterGate() {
	return hardcodedRelativeLaneAngleAfterGate;
}

bool Planner::getUseDistanceToLaneThreshold() {
	return useDistanceToLaneThreshold;
}

double Planner::getRelativeDistanceToLaneThreshold() {
	return relativeDistanceToLaneThreshold;
}

bool Planner::getDoHydrophonesAfterLane() {
	return doHydrophonesAfterLane;
}

double Planner::getEstimatedRelativeAngleForThePinger() {
	return estimatedRelativeAngleForThePinger;
}

bool Planner::getUsingTimerForHydrophonesInsteadOfDistance() {
	return usingTimerForHydrophonesInsteadOfDistance;
}

double Planner::getRelativeDistanceForHydrophonesTask() {
	return relativeDistanceForHydrophonesTask;
}

double Planner::getTimerForHydrophonesTask() {
	return timerForHydrophonesTask;
}

//##############################################################################
// Newly added parameters for the multiple competition scenario:           [END]
//##############################################################################

int get_task_id(std::string name) {
	if (name == "gate") return 1;
	if (name == "lane") return 2;
	if (name == "buoy") return 3;
}

void setRobotInitialPosition(ros::NodeHandle n, int x, int y, int z, int pitch, int roll, int yaw) {
  //TODO: Convert Euler angles to quaternions (?)
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
  if(client.call(setmodelstate)) {
    ROS_INFO("Set robot's position: Success");
  } else {
    ROS_ERROR("Failed to call service ");
  }
}

void setRobotInitialPosition(ros::NodeHandle n, int task_id) {
	switch (task_id) {
		case (1) :
			setRobotInitialPosition(n, 1.58725, -3.98664, 4.5, 0, -0, -3.14159);
			break;
		case (2) :
			setRobotInitialPosition(n, -2.083993, 10.368956, 4.5, 0, 0, -3.141591);
			break;
		case (3) : //TODO: the last parameter is doing nothing for now
			setRobotInitialPosition(n, -1.38, 3.225, 1, 0, 0, -2.3016);
			break;
	}
}

tf::StampedTransform Planner::getStampedTransform(std::string frame) {
	tf::StampedTransform transform;
	tf::TransformListener tf_listener;

	std::string targetFrame = frame; //specified on the setPoints topic
	std::string originalFrame = "robot/rotation_center"; //rpy,xyz of target frame is expressed w.r.t. the original frame axes

	try	{
		double currentTime = ros::Time::now().toSec();
		ROS_DEBUG("waiting for transform- currentTime: %f", currentTime);
		tf_listener.waitForTransform(targetFrame, originalFrame, ros::Time(0), ros::Duration(0.4)); //not sure what an appropriate time to wait is. I wanted to wait less than the target 2 Hz.
		currentTime = ros::Time::now().toSec();
		ROS_DEBUG("done waiting for transform - currentTime: %f", currentTime);
		tf_listener.lookupTransform(targetFrame, originalFrame, ros::Time(0), transform);
	}
	catch (tf::TransformException ex){
		ROS_ERROR("%s",ex.what());
	}
	return transform;
}


bool Planner::areWeThereYet(std::string referenceFrame, std::vector<double> desired) {
	getXYYawFromIMU(referenceFrame);

	//positional bounds
	bool xBounded = fabs(currentX - desired.at(0)) < xBound;
	bool yBounded = fabs(currentY - desired.at(1)) < yBound;
	ROS_DEBUG("Planner::areWeThereYet relative x: %f -- y: %f",	currentX, currentY);

	ROS_INFO_THROTTLE(1, "BOUNDS:");
	ROS_INFO_THROTTLE(1, "xBound: %f    yBound: %f    yawBound: %f", xBound, yBound, yawBound);

	ROS_INFO_THROTTLE(1, "POSITION:");
	ROS_INFO_THROTTLE(1, "Current x: %f    Desired x: %f", currentX, desired.at(0));
	ROS_INFO_THROTTLE(1, "Current y: %f    Desired y: %f", currentY, desired.at(1));

	bool yawBounded = fabs(currentYaw - desired.at(2)) < yawBound;

	ROS_INFO_THROTTLE(1, "ORIENTATION:");
	ROS_INFO_THROTTLE(1, "Current yaw: %f    Desired yaw: %f", currentYaw, desired.at(2));
	ROS_INFO_THROTTLE(1, "Variables bounded: x: %i, y: %i, yaw: %i", xBounded, yBounded, yawBounded);
	return (xBounded && yBounded && yawBounded);
}

void Planner::setVisionObj(int objIndex) {
	planner::CurrentCVTask msgFront;
	planner::CurrentCVTask msgDown;

	msgFront.currentCVTask = msgFront.NOTHING;
	msgDown.currentCVTask = msgDown.NOTHING;
	switch (objIndex) {
	case 0:
		msgFront.currentCVTask = msgFront.NOTHING;
		msgDown.currentCVTask = msgDown.NOTHING;
		break;
	case 1:
		msgFront.currentCVTask = msgFront.GATE;
		msgDown.currentCVTask = msgDown.NOTHING;
		break;
	case 2:
		msgFront.currentCVTask = msgFront.NOTHING;
		msgDown.currentCVTask = msgDown.LANE;
		break;
	case 3:
		msgFront.currentCVTask = msgFront.BUOY;
		msgDown.currentCVTask = msgDown.NOTHING;
		break;
	}

	taskPubFront.publish(msgFront);
	taskPubDown.publish(msgDown);
}

void Planner::setPoints(double pointControl[], std::string referenceFrame) {
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

	msgControl.DepthSpeed.isActive = pointControl[16];
	msgControl.DepthSpeed.data = pointControl[17];

	msgControl.Frame = referenceFrame;

	control_pub.publish(msgControl);
}

void Planner::setVelocity(double x_speed, double y_speed, double yaw_speed, double depth, std::string referenceFrame) {
	double pointControl[18] = { 0, 0, 0, 0, 0, 0, 0, 0, 1, x_speed, 1, y_speed,
			1, yaw_speed, 1, depth, 0, 0};
	setPoints(pointControl, referenceFrame);
}

void Planner::setPosition(std::vector<double> desired, std::string referenceFrame) {
	double pointControl[18] =
			{ 1, desired.at(0), 1, desired.at(1), 1, desired.at(2), 1,
					desired.at(3), 0, 0, 0, 0, 0, 0, 1, desired.at(4), 0, 0 };
	setPoints(pointControl, referenceFrame);
}

void Planner::setVelocityWithCloseLoopYawPitchDepth(double x_speed, double yaw, double pitch, double depth, std::string referenceFrame) {
	double pointControl[18] = { 0, 0, 0, 0, 1, yaw, 1, pitch, 1, x_speed, 0, 0,
			0, 0, 1, depth, 0, 0};
	setPoints(pointControl, referenceFrame);
}

void Planner::setVelocityWithCloseLoopYawPitchOpenLoopDepth(double x_speed, double yaw, double pitch, double depthSpeed, std::string referenceFrame) {
	double pointControl[18] = { 0, 0, 0, 0, 1, yaw, 1, pitch, 1, x_speed, 0, 0,
			0, 0, 0, 0, 1, depthSpeed};
	setPoints(pointControl, referenceFrame);
}

void Planner::setYaw(double yaw, std::string referenceFrame) {
	double pointControl[18] = { 0, 0, 0, 0, 1, yaw, 0, 0, 0, 0, 0, 0,
			0, 0, 0, 0, 0, 0};
	setPoints(pointControl, referenceFrame);
}

void Planner::switchToTask(Tasks newTask) {
	switch(newTask) {
		case Gate:
			delete currentTask;
			currentTask = (Task*) new Task_Gate(this, myStatusUpdater, 0);
			break;
		case Lane:
			delete currentTask;
			currentTask = (Task*) new Task_Lane(this, myStatusUpdater, 0);
			break;
		case Buoy:
			delete currentTask;
			//
			break;
		case Hydrophones:
			delete currentTask;
			//
			break;
		case Kill:
			delete currentTask;
			currentTask = (Task*) new Task_Kill(this, myStatusUpdater, 0);
			break;
	}

	currentTask->execute();
}

void Planner::weAreLost(LostStates newTask, int lostPhase) {
	switch(newTask) {
		case Gate_A:
			delete currentTask;
			currentTask = (Task*) new Lost_Gate(this, myStatusUpdater, lostPhase);
			break;
		case Gate_B:
			delete currentTask;
			//
			break;
		case Lane_A:
			delete currentTask;
			//
			break;
		case Buoy_A:
			delete currentTask;
			//
			break;
		case Hydrophones_A:
			delete currentTask;
			//
			break;
	}

	currentTask->execute();
}

void Planner::resetIMU() {
	ros::ServiceClient seClient = nodeHandle.serviceClient<state_estimation::setInitialPose>("/state_estimation/set_initial_pose");
  	state_estimation::setInitialPose setPose;

	setPose.request.a = 1;

	if(seClient.call(setPose)) {
    	ROS_INFO("notified state estimation: Success");
  	} else {
  		ROS_ERROR("Failed to call state estimation service");
  	}
}

int main(int argc, char **argv) {
	ros::init(argc, argv, "planner");
	ros::NodeHandle nodeHandle;

	ROS_INFO("Planner - Initializing node");
	nodeHandle.param<int>("planner/inSim", inSim, 0);

	Planner* plannerNode = new Planner(nodeHandle);

	ros::Rate loop_rate(10);
	/****This is ros::spin() on a seperate thread*****/
	boost::thread spin_thread(&spinThread);

	///////////////////////////////FOR TESTING/////////////////////////////
	std::string start_task;
	std::string end_task;

	nodeHandle.param<std::string>("planner/start_task", start_task, "gate");
	nodeHandle.param<std::string>("planner/end_task", end_task, start_task);

	int start_task_id = get_task_id(start_task);
	int end_task_id = get_task_id(end_task);

	ROS_INFO("start_task: %s", start_task.c_str());
	ROS_INFO("end_task: %s", end_task.c_str());
	ROS_INFO("start_task_id: %d", start_task_id);
	ROS_INFO("end_task_id: %d", end_task_id);

	if(inSim) {
		setRobotInitialPosition(nodeHandle, start_task_id);
		plannerNode->resetIMU(); //TODO: NOT WORKING ANYMORE JULY 16
	}
	///////////////////////////////////////////////////////////////////////

	//start routine
	plannerNode->switchToTask(plannerNode->Gate);

	return 0;
}

Planner::Planner(ros::NodeHandle& n) {
	go = 0;
	nodeHandle = n;
	estimatedDepth_subscriber = n.subscribe("state_estimation/filteredDepth", 1000, estimatedDepth_callback);
	seeObject_subscriber = n.subscribe("state_estimation/see_object", 1000, &Planner::seeObject_callback, this);

	btClient = n.serviceClient<blinky::UpdatePlannerLights>("update_planner_lights");

	taskPubFront = n.advertise<planner::CurrentCVTask>("currentCVTask_Front", 1000);
	taskPubDown = n.advertise<planner::CurrentCVTask>("currentCVTask_Down", 1000);
	checkpoints_pub = n.advertise<std_msgs::String>("planner/task", 1000);
	control_pub = n.advertise<planner::setPoints>("setPoints", 1000);

	//##########################################################################
	// Fetching the parameter file:
	//##########################################################################
	nodeHandle.param<bool>("openLoopDepth", openLoopDepth, 0);
	nodeHandle.param<double>("openLoopDepthSpeed", openLoopDepthSpeed, 0);
	nodeHandle.param<double>("closeLoopDepth", closeLoopDepth, 0);
	nodeHandle.param<double>("surgeSpeed", surgeSpeed, 0);
	nodeHandle.param<double>("yawTimeout", yawTimeout, 0);
	nodeHandle.param<double>("closeLoopDepthTimeout", closeLoopDepthTimeout, 0);
	nodeHandle.param<double>("openLoopDepthTimeout", openLoopDepthTimeout, 0);
	nodeHandle.param<double>("gateTimeout", gateTimeout, 0);
	nodeHandle.param<double>("laneTimeout", laneTimeout, 0);
	nodeHandle.param<double>("xBound", xBound, 0);
	nodeHandle.param<double>("yBound", yBound, 0);
	nodeHandle.param<double>("yawBound", yawBound, 0);


	//##############################################################################
	// Newly added parameters for the multiple competition scenario:         [START]
	//##############################################################################
	nodeHandle.param<bool>("useCvForLane", useCvForLane, false);
	nodeHandle.param<bool>("useHardcodedLaneAngleAfterGate", useHardcodedLaneAngleAfterGate, false);
	nodeHandle.param<double>("hardcodedRelativeLaneAngleAfterGate", hardcodedRelativeLaneAngleAfterGate, 0.0);
	nodeHandle.param<bool>("useDistanceToLaneThreshold", useDistanceToLaneThreshold, false);
	nodeHandle.param<double>("relativeDistanceToLaneThreshold", relativeDistanceToLaneThreshold, 0.0);
	nodeHandle.param<bool>("doHydrophonesAfterLane", doHydrophonesAfterLane, false);
	nodeHandle.param<double>("estimatedRelativeAngleForThePinger", estimatedRelativeAngleForThePinger, 0.0);
	nodeHandle.param<bool>("usingTimerForHydrophonesInsteadOfDistance", usingTimerForHydrophonesInsteadOfDistance, false);
	nodeHandle.param<double>("relativeDistanceForHydrophonesTask", relativeDistanceForHydrophonesTask, 0.0);
	nodeHandle.param<double>("timerForHydrophonesTask", timerForHydrophonesTask, 0.0);
	//##############################################################################
	// Newly added parameters for the multiple competition scenario:           [END]
	//##############################################################################

	myStatusUpdater = new StatusUpdater(checkpoints_pub, btClient);
	currentTask = new Task(this, myStatusUpdater, 0);

	// Waits until the environment is properly setup until the planner actually starts.
	int ready = 0;
	while (ready == 0) {

		if (seeObject_subscriber.getNumPublishers() == 0) {
			// We need to wait for the publishers to be publishing their data.
			ready = 0;
			ROS_INFO_THROTTLE(2, "Waiting for State Estimation to be ready. Checking /see_object publisher...");
		} else {
			ready = 1;
			ROS_INFO_THROTTLE(1, "State Estimation Ready. Updating Blinky.");
		}

		//TODO: really necessary?
		/*
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
			ROS_DEBUG("Planner - Error thrown in TF listener.");
			ROS_ERROR("%s", ex.what());
		}
		*/
		ROS_DEBUG("Planner - waiting for dependencies");
	}

	myStatusUpdater->updateStatus(myStatusUpdater->ready);

	/*
	 * TODO: Remove everything below once we are ready to test planner, and once we make planner run at startup,
	 * so that the diver doesn't have to unplug the Ethernet cable, which can be hard to do with the new connectors.
	 */
	while (!n.getParam("/go", go) || go != 1) {
		//wait for "go" command from command Lane
    	ROS_INFO_ONCE("Waiting for the 'go' command: rosparam set /go 1");
	}
	/*
	//Set const wait time in header file so diver has time to disconnect Ethernet cable
	ros::Duration(WAIT_TIME_AFTER_GO).sleep();
	*/
}


Planner::~Planner() {

}