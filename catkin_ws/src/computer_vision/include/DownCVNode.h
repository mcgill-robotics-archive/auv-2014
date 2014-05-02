#include "CVNode.h"
#include "MarkerTarget.h"
#include "Line.h"

const std::string DOWN_CV_NODE_NAME = "down_cv_node";

const std::string CAMERA3_CV_TOPIC_NAME = "down_cv_camera";
const std::string OUTPUT_DATA_TOPIC_NAME = "down_cv_data";
const std::string PLANNER_DATA_DOWN_TOPIC_NAME = "currentCVTask_Down";

const int DOWN_CV_NODE_RECEPTION_RATE = 10;
const int DOWN_CV_NODE_BUFFER_SIZE = 1;
bool isUsingHelperWindows;

class DownCVNode : public CVNode {

	public:

	DownCVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	~DownCVNode();

	private:

	cv::Mat* pLastImage;

	void receiveImage(const sensor_msgs::ImageConstPtr& message);
	void listenToPlanner(planner::CurrentCVTask msg);
};
