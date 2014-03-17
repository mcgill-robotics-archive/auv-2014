// This simulates data coming from the planner to the CVNodes.
#include "PlannerSimulator.h"

int main(int argc, char **argv) {

	ros::init(argc, argv, "Planner_Simulator_Node");
	ros::NodeHandle nodeHandle;

	ROS_INFO("Initializing node %s", ros::this_node::getName().c_str());

	// Creates a new CVNode object.
	PlannerSimulator* pPlannerSimulator = new PlannerSimulator(nodeHandle, "Planner_Simulator", 10, 1);

	// Destroy the CVNode object
	delete pPlannerSimulator;

	// Destroy the node
	ros::shutdown();
}

PlannerSimulator::PlannerSimulator(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize) {
	plannerPublisherFront = nodeHandle.advertise<planner::CurrentCVTask>(PLANNER_DATA_FRONT_TOPIC_NAME , 1000); 
	plannerPublisherDown = nodeHandle.advertise<planner::CurrentCVTask>(PLANNER_DATA_DOWN_TOPIC_NAME , 1000); 
	planner::CurrentCVTask msgFront;
	planner::CurrentCVTask msgDown;
	while(1) {
		msgFront.currentCVTask = 1;
		plannerPublisherFront.publish(msgFront);
		sleep(1);
		/*msgFront.currentCVTask = 2;
		plannerPublisherFront.publish(msgFront);
		sleep(1);
		msgFront.currentCVTask = 0;
		plannerPublisherFront.publish(msgFront);
		sleep(1);
		msgDown.currentCVTask = 3;
		plannerPublisherDown.publish(msgDown);
		sleep(1);
		msgDown.currentCVTask = 0;
		plannerPublisherDown.publish(msgDown);
		sleep(1);*/
	}
}
