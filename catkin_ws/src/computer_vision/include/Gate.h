#ifndef CV_DOOR_H
#define CV_GATE_H

#include "VisibleObject.h"
#include "CVNode.h"

// Defines the different constants used:
/**
 * The Kernel size used for the Gaussian Blur filter (used to compensate camera sensor noise).
 */
const int KERNEL_SIZE = 3;

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

/**
 * The Gate class that defines the feature detection filters that will be applied to detect the gate object.
 */
class Gate : public VisibleObject {

public:

	Gate(const CVNode& _parent, const CVNode::GateParameters& _params);
	~Gate();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

private:
	const CVNode::GateParameters params;

	struct PoleCandidate {
		float h, w, rectangleAngleDeg, objectAngleRad, dist;
		cv::Point2f center;
	};

	// Computation functions
	std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV, cv::Scalar start, cv::Scalar end);
	PoleCandidate findRectangleForContour(std::vector<cv::Point>& contour);
	void computePolarCoordinates(PoleCandidate& pole, cv::Point frameCenter, float frameHeight);
	bool handleTwoVisiblePoles(PoleCandidate& p1, PoleCandidate& p2, cv::Point centerOfCurrentFrame);

	// UI functions
	void writePoleCandidateInfo(PoleCandidate& pole, cv::Mat& frame);
	void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour, cv::Scalar COLOR);
	void drawGateInfo(cv::Mat& frame, PoleCandidate& p1, PoleCandidate& p2);

	// Information on the gate
	bool m_isVisible;
	double m_yawAngle;
	double m_xDistance;
	double m_yDistance;
	double m_zDistance;

	// Trackbar-accessible variables
	int trackbar_start_hue_threshold;
	int trackbar_end_hue_threshold;
	int trackbar_start_val_threshold;
	int trackbar_end_val_threshold;

protected:

	void applyFilter(cv::Mat& currentFrame);
};

#endif
