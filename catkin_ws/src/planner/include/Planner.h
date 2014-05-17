#ifndef Planner_h
#define Planner_h

#include "StatusUpdater.h"
#include "ros/ros.h"
#include "std_msgs/Float64.h"
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
#include <vector>
#include <cmath>
#include <boost/thread.hpp>

/**
 * The duration of time the planner will wait for the TF broadcaster to setup before timing out. (in seconds I think)
 */
int const TF_BROADCASTER_TIMOUT_PERIOD = 10;

void spinThread();
void estimatedDepth_callback(const std_msgs::Float64 msg);
int get_task_id(std::string name);
void setRobotInitialPosition(ros::NodeHandle n, int x, int y, int z, int pitch, int roll, int yaw);
void setRobotInitialPosition(ros::NodeHandle n, int task_id);


class Task;

class Planner{

	public:
	enum Tasks {Gate, Lane, Buoy, Hydrophones, Kill};
	double getDepth();
	bool areWeThereYet(std::string referenceFrame, std::vector<double> desired);
	void setVisionObj(int objIndex);
	void setPosition(std::vector<double> desired, std::string referenceFrame);
	void setVelocity(double x_speed, double y_speed, double yaw_speed, double depth, std::string referenceFrame);
	void switchToTask(Tasks newTask);
	Planner(ros::NodeHandle& nodeHandle);
	//~Planner();

	private:
	void setTransform(std::string referenceFrame);
	void setPoints(double pointControl[], std::string referenceFrame);

	int ready;
	int go;
	ros::Subscriber estimatedDepth_subscriber;

	ros::ServiceClient btClient;

	ros::Publisher wrench_pub;
	ros::Publisher CV_objs_pub;
	ros::Publisher control_pub;
	ros::Publisher checkpoints_pub;
	ros::Publisher taskPubFront;
	ros::Publisher taskPubDown;

	Task* currentTask;
	StatusUpdater* myStatusUpdater;

	int xBound;
	int yBound;
	int zBound;
	int yawBound;
	int pitchBound;
};

#endif