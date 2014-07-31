#ifndef Planner_h
#define Planner_h

#include "StatusUpdater.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Pose.h"
#include "geometry_msgs/PoseStamped.h"
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/SetModelState.h"
#include "tf/transform_listener.h"
#include "planner/setPoints.h"
#include "planner/CurrentCVTask.h"
#include "blinky/RGB.h"
#include "blinky/UpdatePlannerLights.h"
#include "hydrophones/StartHydrophoneTask.h"
#include "state_estimation/setInitialPose.h"
#include <vector>
#include <cmath>
#include <boost/thread.hpp>

/**
 * The duration of time the planner will wait for the TF broadcaster to setup before timing out. (in seconds I think)
 */
int const TF_BROADCASTER_TIMOUT_PERIOD = 10;

int const WAIT_TIME_AFTER_GO = 20; //wait time between sending "go" and assuming Ethernet cable has been unplugged

void spinThread();
void estimatedDepth_callback(const std_msgs::Float64 msg);
int get_task_id(std::string name);
void setRobotInitialPosition(ros::NodeHandle n, int x, int y, int z, int pitch, int roll, int yaw);
void setRobotInitialPosition(ros::NodeHandle n, int task_id);

class Task;

class Planner{

	public:
	enum Tasks {Gate, Lane, Buoy, Hydrophones, Kill};
	enum LostStates {Gate_A, Gate_B, Lane_A, Buoy_A, Hydrophones_A};
	double getDepth();
	double getCurrentYaw(std::string frame);
	double getCurrentX(std::string frame);
	double getCurrentY(std::string frame);

	bool getSeeObject();
	bool isOpenLoopDepth();
	double getOpenLoopDepthSpeed();
	double getCloseLoopDepth();
	double getSurgeSpeed();
	double getYawTimeout();
	double getCloseLoopDepthTimeout();
	double getOpenLoopDepthTimeout();
	double getGateTimeout();
	double getLaneTimeout();
	double getYawBound();

	// Newly added getters for the multiple competition scenarios:
	bool getUseCvForLane();
	bool getUseHardcodedLaneAngleAfterGate();
	double getHardcodedRelativeLaneAngleAfterGate();
	bool getUseDistanceToLaneThreshold();
	double getRelativeDistanceToLaneThreshold();
	bool getDoHydrophonesAfterLane();
	double getEstimatedRelativeAngleForThePinger();
	bool getUsingTimerForHydrophonesInsteadOfDistance();
	double getRelativeDistanceForHydrophonesTask();
	double getTimerForHydrophonesTask();

	bool areWeThereYet(std::string referenceFrame, std::vector<double> desired);
	void setVisionObj(int objIndex);
	void setPosition(std::vector<double> desired, std::string referenceFrame);
	void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth, std::string referenceFrame);
	void setVelocityWithCloseLoopYawPitchDepth(double x_speed, double yaw, double pitch, double depth, std::string referenceFrame);
	void setVelocityWithCloseLoopYawPitchOpenLoopDepth(double x_speed, double yaw, double pitch, double depthSpeed, std::string referenceFrame);
	void setYaw(double yaw, std::string referenceFrame);

	void getXYYawFromIMU(std::string frame);
	void switchToTask(Tasks newTask);
	void weAreLost(LostStates newTask, int lostPhase);
	void resetIMU();

    void startHydrophones();

	tf::StampedTransform getStampedTransform(std::string referenceFrame);

	Planner(ros::NodeHandle& nodeHandle);
	~Planner();

	private:
	void setPoints(double pointControl[], std::string referenceFrame);
	void seeObject_callback(const std_msgs::Bool msg);

	double currentYaw;
	double currentX;
	double currentY;

	bool seeObject;
	bool openLoopDepth;
	double openLoopDepthSpeed;
	double closeLoopDepth;
	double surgeSpeed;
	double yawTimeout;
	double closeLoopDepthTimeout;
	double openLoopDepthTimeout;
	double gateTimeout;
	double laneTimeout;

	// Newly added parameters for the multiple competition scenario
	bool useCvForLane;
	bool useHardcodedLaneAngleAfterGate;
	double hardcodedRelativeLaneAngleAfterGate;
	bool useDistanceToLaneThreshold;
	double relativeDistanceToLaneThreshold;
	bool doHydrophonesAfterLane;
	double estimatedRelativeAngleForThePinger;
	bool usingTimerForHydrophonesInsteadOfDistance;
	double relativeDistanceForHydrophonesTask;
	double timerForHydrophonesTask;

	ros::NodeHandle nodeHandle;
	int go;
	ros::Subscriber estimatedDepth_subscriber;
	ros::Subscriber seeObject_subscriber;

	ros::ServiceClient btClient;
	ros::ServiceClient startHydrophonesClient;

	ros::Publisher wrench_pub;
	ros::Publisher control_pub;
	ros::Publisher checkpoints_pub;
	ros::Publisher taskPubFront;
	ros::Publisher taskPubDown;

	Task* currentTask;
	StatusUpdater* myStatusUpdater;

	double xBound;
	double yBound;
	double zBound;
	double yawBound;
	double pitchBound;
};

#endif
