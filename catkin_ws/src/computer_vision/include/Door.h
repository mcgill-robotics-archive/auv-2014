#ifndef CV_DOOR_H
#define CV_DOOR_H

#include "VisibleObject.h"

// Defines the different constants used:
const int CANNY_RATIO = 3;
const int CANNY_LOW_THRESHOLD = 200;
const int KERNEL_SIZE = 3;
const int GATE_RATIO = 16; // It's 1:16 for the width and height.
const int GATE_RATIO_ERROR = 5;
const float FOCAL_LENGTH = 8;
const float DOOR_REAL_HEIGHT = 1219.2;
const float CAMERA_SENSOR_HEIGHT = 6.26;
const int ESCAPE_KEY = 27;
const int TIME_BETWEEN_FRAME = 150; // This is in miliseconds.
const int MIN_NUMBER_EDGES = 4;
const int MAX_NUMBER_EDGES = 50;

// Defines the basic colors used in the BGRX color space.
const cv::Scalar GREEN_BGRX = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE_BGRX = cv::Scalar(255, 0, 0);
const cv::Scalar RED_BGRX = cv::Scalar(0, 0, 255);
const cv::Scalar WHITE_BGRX = cv::Scalar(255, 255, 255);
const cv::Scalar MAUVE_BGRX = cv::Scalar(212, 115, 212);
const cv::Scalar HSV_STARTING_FILTER_RANGE = cv::Scalar(0, 0, 40);
const cv::Scalar HSV_ENDING_FILTER_RANGE = cv::Scalar(20, 255, 220);

// TO BE REMOVED (FOR DESIGN PROJECT PRESENTATION ONLY)
const char* FILTERED_WINDOW = "Filtered Feed";

// Door class
class Door : public VisibleObject {

	public:

	~Door();
	ObjectData* retrieveObjectData(cv::Mat& currentFrame);

	private:

	void applyFilter(cv::Mat& currentFrame);
	std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV);
	void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour);
};

#endif

