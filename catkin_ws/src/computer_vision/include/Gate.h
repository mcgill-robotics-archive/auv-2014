#ifndef CV_DOOR_H
#define CV_GATE_H

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
 * The real ration is 16:1. If the ratio is less than 4:1, it's probably not the gate.
 */
const int MIN_GATE_RATIO = 4;
/**
 * The deviation tolerance
 */
int gate_ratio_error = 5;

/**
 * The angle at which rectangles should be.
 */
const int DESIRED_ANGLE_OF_RECT = 90;

/**
 * The accepted error for the angle of a rectangle.
 */
const int ANGLE_RECT_ERROR = 5;

/**
 * The focal length of the lenses.
 */
const float FOCAL_LENGTH = 8;
/**
 * The height of the gate in meters.
 */
const float DOOR_REAL_HEIGHT = 1.2;
/**
 * The distance between the cylinders' centers in meters.
 */
const float DISTANCE_BETWEEN_ORANGE_CYLINDER = 3.1262;
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
const int MIN_NUMBER_POINTS = 4;
/**
 * The maximum number of edges that a shape must have in order to be accepted for further filtering.
 */
int max_number_points = 100;

/**
 * The end of the threshold for the hue.
 */
int end_hsv_hue_threshold = 20;

/**
 * The end of the threshold for the value.
 */
int end_hsv_value_threshold = 220;

/**
 * The start of the threshold for the value.
 */
int start_hsv_value_threshold = 40;

/**
 * The threshold used for approximating polygons.
 */
int polygon_approximation_threshold = 3;

/**
 * The maximum range that the HSV values can take.
 */
const int MAX_HSV_VALUE = 255;

// Defines the basic colors used in the BGRX color space.
const cv::Scalar GREEN_BGRX = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE_BGRX = cv::Scalar(255, 0, 0);
const cv::Scalar RED_BGRX = cv::Scalar(0, 0, 255);
const cv::Scalar WHITE_BGRX = cv::Scalar(255, 255, 255);
const cv::Scalar MAUVE_BGRX = cv::Scalar(212, 115, 212);
cv::Scalar HSV_STARTING_FILTER_RANGE = cv::Scalar(0, 0, start_hsv_value_threshold);
cv::Scalar HSV_ENDING_FILTER_RANGE = cv::Scalar(end_hsv_hue_threshold, 255, end_hsv_value_threshold);

// TO BE REMOVED (FOR DESIGN PROJECT PRESENTATION ONLY)
const char* FILTERED_WINDOW = "Filtered Feed";

/**
 * The Gate class that defines the gate visible object.
 */
class Gate : public VisibleObject {

public:

	Gate();
	~Gate();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

private:

	void applyFilter(cv::Mat& currentFrame);
	std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV);
	void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour, cv::Scalar COLOR);

	struct PoleCandidate {
		float h, w, angle, dist;
		cv::Point2f center;
	};

	bool m_isVisible;
	double m_yawAngle;
	double m_xDistance;
	double m_yDistance;
	double m_zDistance;
};

#endif
