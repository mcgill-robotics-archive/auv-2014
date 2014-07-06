#include "CVNode.h"
#include "MarkerTarget.h"
#include "Line.h"

const std::string DOWN_CV_NODE_NAME = "down_cv_node";
const std::string CAMERA3_CV_TOPIC_NAME = "down_cv/camera1";
const std::string OUTPUT_DATA_TOPIC_NAME = "down_cv/data";
const std::string PLANNER_DATA_DOWN_TOPIC_NAME = "currentCVTask_Down";

const int DOWN_CV_NODE_RECEPTION_RATE = 10;
const int DOWN_CV_NODE_BUFFER_SIZE = 1;

class DownCVNode : public CVNode {

	public:

	DownCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	~DownCVNode();

	private:

	cv::Mat* pLastImage;

	void imageHasBeenReceived(const sensor_msgs::ImageConstPtr& message);
	cv::Mat convertSensorMessageToOpencvImage(const sensor_msgs::ImageConstPtr& message);
	void listenToPlanner(planner::CurrentCVTask msg);
};
