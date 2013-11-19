#include <cv_bridge/cv_bridge.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv/cv.h>
#include <iostream>
#include <cmath>

#include "ObjectData.h"

class VisibleObject {

	public:

	virtual ObjectData* retrieveObjectData(cv::Mat& currentFrame) = 0;
	cv::Mat convertFromBGRXToHSV(const cv::Mat& currentFrame);

	private:

	virtual void applyFilter(cv::Mat& currentFrame) = 0;
};
