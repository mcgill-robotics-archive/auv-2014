#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>

#include "Door.h"

const char* MAIN_WINDOW = "Camera Feed";

class CVNode {

	private:
	
	image_transport::ImageTransport* pImageTransport;
	image_transport::Subscriber subscriber;
	std::list<VisibleObject*> visibleObjects;
	
	public:
	
	CVNode(ros::NodeHandle& nodeHandle, const char* topicName);
	~CVNode();
	int getNumPublisher();
	
	private:
	
	void receiveImage(const sensor_msgs::ImageConstPtr& message);
};
	


