#ifndef CV_DOOR_H
#define CV_GATE_H

#include "VisibleObject.h"
#include "CVNode.h"

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
const int ANGLE_RECT_ERROR = 15;

/**
 * The height of the gate in meters.
 */
const float DOOR_REAL_HEIGHT = 1.2;
/**
 * The distance between the cylinders' centers in meters.
 */
const float GATE_WIDTH = 3.1262;

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
 * The threshold used for approximating polygons.
 */
int polygon_approximation_threshold = 3;

/**
 * The maximum range that the HSV values can take.
 */
const int MAX_HSV_VALUE = 255;
const int MAX_HSV_HUE = 179; //In OpenCV, hue can take 180 values (0 to 179)

// Defines the basic colors used in the BGRX color space.
const cv::Scalar GREEN_BGRX = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE_BGRX = cv::Scalar(255, 0, 0);
const cv::Scalar RED_BGRX = cv::Scalar(0, 0, 255);
const cv::Scalar WHITE_BGRX = cv::Scalar(255, 255, 255);
const cv::Scalar MAUVE_BGRX = cv::Scalar(212, 115, 212);

// TO BE REMOVED (FOR DESIGN PROJECT PRESENTATION ONLY)
const char* FILTERED_WINDOW = "Filtered Feed";

/**
 * The Gate class that defines the feature detection filters that will be applied to detect the gate object.
 */
class Gate : public VisibleObject {

public:

	Gate(const CVNode& _parent);
	~Gate();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

private:
	struct PoleCandidate {
		float h, w, rectangleAngleDeg, objectAngleRad, dist;
		cv::Point2f center;
	};

	std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV, cv::Scalar start, cv::Scalar end);
	PoleCandidate findRectangleForContour(std::vector<cv::Point>& contour);
	void computePolarCoordinates(PoleCandidate& pole, cv::Point frameCenter, float frameHeight);
	bool handleTwoVisiblePoles(PoleCandidate& p1, PoleCandidate& p2, cv::Point centerOfCurrentFrame);

	void writePoleCandidateInfo(PoleCandidate& pole, cv::Mat& frame);
	void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour, cv::Scalar COLOR);
	void drawGateInfo(cv::Mat& frame, PoleCandidate& p1, PoleCandidate& p2);

	bool m_isVisible;
	double m_yawAngle;
	double m_xDistance;
	double m_yDistance;
	double m_zDistance;

	int trackbar_start_hue_threshold;
	int trackbar_end_hue_threshold;
	int trackbar_start_val_threshold;
	int trackbar_end_val_threshold;

protected:

	void applyFilter(cv::Mat& currentFrame);
};

#endif
