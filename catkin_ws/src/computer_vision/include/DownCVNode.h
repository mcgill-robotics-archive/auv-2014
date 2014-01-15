#include "CVNode.h"

class DownCVNode : public CVNode {

	public:

	DownCVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate);

	private:

	void receiveImage(const sensor_msgs::ImageConstPtr& message, const std::string &topicName);
};
