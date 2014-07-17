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
const int ONE_SECOND = 1;

class CVNode {

	public:
	
	CVNode(ros::NodeHandle& nodeHandle, std::string topicName, int receptionRate, int bufferSize);
	virtual ~CVNode();
	void receiveImages();

	inline double get_camera_sensor_height() const { return camera_sensor_height; }
	inline double get_camera_focal_length() const { return camera_focal_length; }
	inline bool get_down_using_helpers() const { return down_using_helpers; }
	inline bool get_front_using_helpers() const { return front_using_helpers; }

	struct GateParameters {
		int min_number_points_contour;

		int pole_desired_angle_deg;		
		int pole_angle_error_deg;

		double gate_width_error_mSqr;

		int min_pole_ratio;

		double min_convexity_ratio;

		int gate_height_m;
		int gate_width_m;

		int value_range_begin;
		int value_range_end;

		int hue_range1_begin;
		int hue_range1_end;
		int hue_range2_begin;
		int hue_range2_end;
	};

	struct LineParameters {
		// Range 1:
		int hue_range1_begin;
		int hue_range1_end;
		int sat_range1_begin;
		int sat_range1_end;
		int value_range1_begin;
		int value_range1_end;

		// Range 2:
		int hue_range2_begin;
		int hue_range2_end;
		int sat_range2_begin;
		int sat_range2_end;
		int value_range2_begin;
		int value_range2_end;

		// Range 3:
		int hue_range3_begin;
		int hue_range3_end;
		int sat_range3_begin;
		int sat_range3_end;
		int value_range3_begin;
		int value_range3_end;

		// Range 4:
		int hue_range4_begin;
		int hue_range4_end;
		int sat_range4_begin;
		int sat_range4_end;
		int value_range4_begin;
		int value_range4_end;
	};

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
	std::list<VisibleObject*> objectsToSearchFor;
	int numFramesWithoutObject;

	double camera_sensor_height;
	double camera_focal_length;
	bool down_using_helpers;
	bool front_using_helpers;

	GateParameters gp;
	LineParameters lp;

	private:

	virtual void imageHasBeenReceived(const sensor_msgs::ImageConstPtr& message) = 0;
};
	
#endif
