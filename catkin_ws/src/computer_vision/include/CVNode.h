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
#include "planner/CurrentCVTask.h"
#include "VisibleObject.h"
#include <opencv2/imgproc/imgproc.hpp>

#define DELAY_BETWEEN_INFOS 5

const std::string VIDEO_FEED_TOPIC_NAME = "camera_feed";
const std::string FORWARD_CAMERAS_TOPIC_NAME = "forward_cameras_object";
const int FRAME_VISIBILITY_THRESHOLD = 10;

class CVNode {

	private:
	
	/**
	 * Defines the rate at which the node will be checking for incoming messages.
	 */
	int receptionRate;
	image_transport::Subscriber cameraNodeSubscriber;

	protected:

	image_transport::ImageTransport* pImageTransport;
	image_transport::Publisher frontEndPublisher;
	ros::Publisher frontEndVisibleObjectDataPublisher;
	ros::Subscriber plannerSubscriber;
	std::list<VisibleObject*> visibleObjectList;
	int numFramesWithoutObject;

	int start_hsv_hue_thresh;
	int end_hsv_hue_thresh;
	int start_hsv_value_thresh;
	int end_hsv_value_thresh;
	double camera_sensor_height;
	double camera_focal_length;
	bool down_using_helpers;
	bool front_using_helpers;
	
	public:
	
	CVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	virtual ~CVNode();
	void receiveImages();

	inline int get_start_hsv_hue_thresh() const { return start_hsv_hue_thresh; }
	inline int get_end_hsv_hue_thresh() const { return end_hsv_hue_thresh; }
	inline int get_start_hsv_value_thresh() const { return start_hsv_value_thresh; }
	inline int get_end_hsv_value_thresh() const { return end_hsv_value_thresh; }
	inline double get_camera_sensor_height() const { return camera_sensor_height; }
	inline double get_camera_focal_length() const { return camera_focal_length; }
	inline bool get_down_using_helpers() const { return down_using_helpers; }
	inline bool get_front_using_helpers() const { return front_using_helpers; }

	struct GateParameters {
		int min_number_points_contour;

		int pole_desired_angle_deg;		
		int pole_angle_error_deg;

		int min_pole_ratio;

		int gate_height_m;
		int gate_width_m;
	};

	protected:
	GateParameters gp;

	private:

	virtual void receiveImage(const sensor_msgs::ImageConstPtr& message) = 0;
};
	
#endif
