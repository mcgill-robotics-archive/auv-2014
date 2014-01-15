#ifndef CV_DOOR_H
#define CV_DOOR_H

#include "VisibleObject.h"

// Defines the different constants used:
/**
 * The ratio used in the canny transform to determine shape contours.
 */
const int CANNY_RATIO = 3;
/**
 * The lowest threshold used in the canny transform to determine shape contours.
 */
const int CANNY_LOW_THRESHOLD = 200;
/**
 * The Kernel size used for the Gaussian Blur filter (used to compensate camera sensor noise).
 */
const int KERNEL_SIZE = 3;
/**
 * The gate's orange height/width ratio used to filter false positive readings on a frame.
 */
const int GATE_RATIO = 16; // It's 1:16 for the width and height.
/**
 * The deviation tolerance
 */
const int GATE_RATIO_ERROR = 5;
/**
 * The focal length of the lenses.
 */
const float FOCAL_LENGTH = 8;
/**
 * The height of the gate in millimeters.
 */
const float DOOR_REAL_HEIGHT = 1219.2;
/**
 * The eight of the sensor of the camera in millimeters.
 */
const float CAMERA_SENSOR_HEIGHT = 6.26;
/**
 * The escape key's ID.
 */
const int ESCAPE_KEY = 27;
/**
 * The delay between each frame on the stream, in milliseconds.
 */
const int TIME_BETWEEN_FRAME = 150; // This is in milliseconds.
/**
 * The minimum number of edges that a shape must have in order to be accepted for further filtering.
 */
const int MIN_NUMBER_EDGES = 4;
/**
 * The maximum number of edges that a shape must have in order to be accepted for further filtering.
 */
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
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

	private:

	void applyFilter(cv::Mat& currentFrame);
	std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV);
	void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour);
};

#endif

