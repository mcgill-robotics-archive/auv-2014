#ifndef CV_VISIBLE_OBJECT_H
#define CV_VISIBLE_OBJECT_H

#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <cmath>
#include "computer_vision/VisibleObjectData.h"

class VisibleObject {

	public:

	virtual std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame) = 0;
	cv::Mat convertFromBGRXToHSV(const cv::Mat& currentFrame);

	virtual ~VisibleObject();

	private:

	virtual void applyFilter(cv::Mat& currentFrame) = 0;
};

#endif
