#ifndef CV_VISIBLE_OBJECT_H
#define CV_VISIBLE_OBJECT_H

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <cmath>
#include <math.h>
#include <boost/lexical_cast.hpp>
#include "computer_vision/VisibleObjectData.h"

class CVNode; //Forward-declaration of CVNode, because CVNode.h includes this file

class VisibleObject {

public:

	VisibleObject(const CVNode& _parent);
	virtual std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame) = 0;
	cv::Mat convertFromBGRXToHSV(const cv::Mat& currentFrame);
	virtual ~VisibleObject() = 0;

protected:

	const CVNode& parent;
	virtual void applyFilter(cv::Mat& currentFrame) = 0;
};

#endif
