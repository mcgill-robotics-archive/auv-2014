#ifndef CV_NODE_H
#define CV_NODE_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <list>
#include "computer_vision/VisibleObjectData.h"

#include "VisibleObject.h"
#include <opencv2/imgproc/imgproc.hpp>

class CVNode {

	private:
	
	/**
	 * Defines the rate at which the node will be checking for incoming messages.
	 */
	int receptionRate;
	std::list<image_transport::Subscriber> subscribers;

	protected:

	image_transport::ImageTransport* pImageTransport;
	image_transport::Publisher publisher;
	std::list<VisibleObject*> visibleObjects;

	public:
	
	CVNode(ros::NodeHandle& nodeHandle, std::list<std::string> topicList, int receptionRate);
	~CVNode();
	void receiveImages();
	
	private:
	
	virtual void receiveImage(const sensor_msgs::ImageConstPtr& message, const std::string &topicName) = 0;
};

std::string getTopicName(image_transport::Subscriber subscriber);
	
#endif
