#include "VisibleObject.h"

VisibleObject::VisibleObject(const CVNode& _parent) : parent(_parent) {
	//Initialization of the parent member was done in the init. list.
}

/**
 * Converts the cv::Mat object passed in parameter and converts it from BGRX to HSV color space.
 *
 * @param currentFrame The cv::Mat object to be returned.
 * @return The converted cv::Mat object in HSV color space.
 */
cv::Mat VisibleObject::convertFromBGRXToHSV(const cv::Mat& currentFrame) {
	cv::Mat currentFrameInHSV;
	cv::cvtColor(currentFrame, currentFrameInHSV, CV_BGR2HSV);

	return (currentFrameInHSV);
}

/**
 * Destructor.
 */
VisibleObject::~VisibleObject() {
	/* There is an empty destructor here because the VisibleObject destructor has to be virtual,
	 * since we are using polymorphic pointers of VisibleObjects. */
}
