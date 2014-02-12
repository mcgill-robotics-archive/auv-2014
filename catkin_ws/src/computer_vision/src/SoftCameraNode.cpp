//Get relevant topic names
#include "DownCameraNode.h"
#include "FrontCameraNode.h"

#include "SoftCameraNode.h"

#include <opencv2/highgui/highgui.hpp>
#include <cstring>

int main(int argc, char* argv[])
{
	//Arguments: video path, publish on down/front
	bool wrong = (argc != 3), isDown = false, isFront = false;
	if(!wrong) {
		isDown = (strcmp(argv[2], "down") == 0);
		isFront = (strcmp(argv[2], "front") == 0);
		wrong = !isDown && !isFront;
	}

	if(wrong) {
		ROS_ERROR("Usage: rosrun computer_vision SoftCameraNode <video path> <down | front>");
		return 1;
	}

	ros::init(argc, argv, SOFT_CAMERA_NODE_NAME);
	ros::NodeHandle nodeHandle;
	cv::VideoCapture video(argv[1]);
	if (!video.isOpened()) {
		ROS_ERROR("Error: Video path is not valid.");
		return 1;
	}
	std::string topic = isDown ? DOWN_CAMERA_NODE_TOPIC : FRONT_CAMERA_NODE_TOPIC;

	SoftCameraNode node(nodeHandle, topic, video);
	node.sendImages();
	
	ros::shutdown();
	
	return 0;
}

SoftCameraNode::SoftCameraNode(ros::NodeHandle& nodeHandle, const std::string topicName, const cv::VideoCapture& video) 
: CameraNode(nodeHandle, topicName, SOFT_CAMERA_NODE_TRANSMISSION_RATE, SOFT_CAMERA_NODE_BUFFER_SIZE)
{
	videoSource = video;
	cv::namedWindow( SOFT_CAMERA_NODE_NAME.c_str(), CV_WINDOW_AUTOSIZE );
}

SoftCameraNode::~SoftCameraNode() 
{
	cv::destroyWindow( SOFT_CAMERA_NODE_NAME.c_str() );
}

void SoftCameraNode::sendImages() 
{
	ros::Rate loop_rate(transmissionRate);
	cv::Mat frame;
	while (ros::ok()) {
  		
		//Obtain frame from the video	
		if(!videoSource.grab()) {
			break;
		}
  		if(!videoSource.retrieve(frame)) {			
			break;
		}

		cv::imshow(SOFT_CAMERA_NODE_NAME.c_str(), frame);
	
		// Publish the new frame
		try {
			cvNodePublisher.publish(toCvImage(&frame).toImageMsg());
		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("CvBridge exception in DownCameraNode::sendImages(): %s", e.what());
			break;
		}

		ros::spinOnce();
		loop_rate.sleep();
  	}
}
