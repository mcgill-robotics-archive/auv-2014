/*
 * MarkerTarget.h
 *
 *  Created on: Nov 20, 2013
 *      Author: fred
 */

#ifndef CV_MARKER_TARGET_H
#define CV_MARKER_TARGET_H

#include "VisibleObject.h"

/* Marker target: a target (image of some kind) placed in a bin at the bottom of the pool.
 * Two markers are to be dropped, one in the primary and one in the secondary bin. */

enum ObjectType { UNDEFINED = -1, DOOR, BOEY, GROUND_TARGET_1, GROUND_TARGET_2, GROUND_TARGET_3, GROUND_TARGET_4 };

class MarkerTarget : public VisibleObject
{
public:
	MarkerTarget();

	virtual computer_vision::VisibleObjectData* retrieveObjectData(cv::Mat& currentFrame);
	virtual void applyFilter(cv::Mat& currentFrame);
private:
	typedef std::vector<std::vector<cv::Point2f> > Point2DVec;

	/* Small structure that describes a detected marker in the image. */
	struct MarkerDescriptor {
		std::vector<cv::Point2f> imageCorners; //The corners of the marker (i.e. the corners of the bin)
		ObjectType closestMatch; //Which marker template is the closest to this one
		long diffFromMatch; //How close is the template to this one
		float yaw_angle, pitch_angle; //Angles between us and the marker, in degrees
		float x_dist, y_dist, z_dist; //Distances between us and the marker, in centimeters

		MarkerDescriptor(const std::vector<cv::Point2f>& _imageCorners, ObjectType _closestMatch, long _diffFromMatch) :
			imageCorners(_imageCorners),
			closestMatch(_closestMatch),
			diffFromMatch(_diffFromMatch),
			yaw_angle(0.f),
			pitch_angle(0.f),
			x_dist(0.f),
			y_dist(0.f),
			z_dist(0.f)
		{ }
	};

	/* Reference markers. Each marker is stored twice to account for the two possible
	 * configurations (normal, and flipped on both axes) */
	const static int NUM_REFERENCE_IMGS = 8;
	cv::Mat referenceImgs[NUM_REFERENCE_IMGS];

	const static int CANNY_THRESHOLD = 100;
	const static int ADAPTIVE_THRESHOLD_KERNEL_SIZE = 41;


	/* Maximum allowed difference between a captured image and a reference image, in pixels*/
	int errorTolerance;

	/* Intrinsic camera parameters, used for 3D pose estimation */
	cv::Mat_<float> intrinsic;

	// ===== Bin-finding functions =====
	/* Finds bins on the bottom of the pool. Returns a list of bin corners, grouped by 4.*/
	Point2DVec findBins(cv::Mat& frame);

	/* Creates a contour map of the current frame. outContoursPoly contains the polygons associated to each contour.*/
	cv::Mat getContourMap(cv::Mat& frame, Point2DVec& outContoursPoly);

	/* Obtains a list of the corners that could be bins. */
	Point2DVec getPreliminaryCorners(const cv::Mat& contourMap, const Point2DVec& contoursPoly);

	/* Modifies the corners so that they are in the correct order for marker detection. */
	void refineCorners(Point2DVec& corners);

	/* Checks whether a rectangle has the correct 2:1 ratio for us to consider it's a bin.*/
	bool checkRectangleRatio(const cv::RotatedRect& rectangle);

	// ===== Marker-finding functions =====
	/* From a list of bins, determines which markers are inside them. */
	std::vector<MarkerDescriptor> findMarkers(const cv::Mat& frame, const Point2DVec& bins);

	/* Computes the number of differences between a marker candidate (in the captured image) and a marker reference. */
	long compareImages(const std::vector<cv::Point2f> &sourceCoords, const cv::Mat& cap, const cv::Mat& ref);

	// ===== Pose estimation functions =====
	/* Given a MarkerDescriptor, estimate its distance and angles relative to us. */
	void estimatePose(MarkerDescriptor& inOutMarker);
};

#endif /* CV_MARKER_TARGET_H */
