/*
 * MarkerTarget.cpp
 *
 * @author Fred Lafrance
 * @author Michael Noseworthy
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
#include <ros/package.h>
#include <ros/console.h>
#include "MarkerTarget.h"

#define PI 3.141592653589793

/**
 * The constructor loads the reference images which we will use to compare our candidate markers to.
 * Also initializes the matrix we will use to determine the positions of the objects.
 */
MarkerTarget::MarkerTarget() {

	std::string templatePath = ros::package::getPath("computer_vision") +
			"/reference_markers/%d.png";

	char *path = new char[templatePath.size()];
	//Load the reference images
	for(int i = 0 ; i < NUM_REFERENCE_IMGS; i += 2) {
		sprintf(path, templatePath.c_str(), (i + 2) / 2);

		referenceImgs[i] = cv::imread(path, CV_LOAD_IMAGE_GRAYSCALE);
		cv::flip(referenceImgs[i], referenceImgs[i + 1], -1);
	}

	delete[] path;

	/* To set the error tolerance, assume that all reference images
	 * have the same size. */
	//errorTolerance = (int) (((float) (referenceImgs[0].cols * referenceImgs[0].rows)) * 0.15f);
	errorTolerance = 20000;
	/* Create the intrinsic camera maxtrix, used to estimate pose. 
	 * NOTE: These values will only work properly with the simulator! */
	intrinsic = cv::Mat(3, 3, 0); //Skew and other entries are 0
	intrinsic.at<float>(0, 0) = 0.02f; //Focal length x corresponds to near plane
	intrinsic.at<float>(1, 1) = 0.02f; //Same with focal length y
	intrinsic.at<float>(2, 2) = 1; //Always 1
	//Principal point, in pixels. Use center of the video feed (simulator: (400, 400))
	intrinsic.at<float>(0, 2) = 400;
	intrinsic.at<float>(1, 2) = 400;
}

/**
 * Detects if any markers are present and if so returns their position, and the identity of the marker object.
 *
 * @param currentFrame The current camera frame in which to look for the marker bins.
 * @return A vector (std::vector< VisibleObjectData* >) of VisibleDataObjects populated with information such 
 *				as distance, angle, and marker type for each visible marker bin.
 */
std::vector<computer_vision::VisibleObjectData*> MarkerTarget::retrieveObjectData(cv::Mat& currentFrame) {
	applyFilter(currentFrame);
	cv::Mat filteredFrame = currentFrame.clone();

	Point2DVec binCorners = findBins(currentFrame);
	std::vector<MarkerDescriptor> markers = findMarkers(filteredFrame, binCorners);

	for(unsigned int i = 0 ; i < markers.size() ; i++)
		estimatePose(markers[i]);
	
	std::vector<computer_vision::VisibleObjectData*> messages;
	//TODO construct the object with the data in each marker descriptor and return it
	if (markers.size() > 0) {
		//TODO Decide what to do if we detect more than one marker
		for (int i = 0; i < markers.size(); i++) {
			computer_vision::VisibleObjectData* objectData = new computer_vision::VisibleObjectData();
			objectData->object_type = markers[i].closestMatch;
			objectData->x_distance = markers[i].x_dist;
			objectData->y_distance = markers[i].y_dist;
			objectData->z_distance = markers[i].z_dist;
			objectData->pitch_angle = markers[i].pitch_angle;
			objectData->yaw_angle = markers[i].yaw_angle;
			messages.push_back(objectData);
		}
	}
	cv::cvtColor( currentFrame, currentFrame, CV_GRAY2BGR );
	return messages;
}

/**
 * Converts the image to grayscale since we are only looking for edges amongst a black and white surface.
 * Also blurs the image to remove noise.
 *
 * @param currentFrame The camera frame in which to apply the filter.
 */
void MarkerTarget::applyFilter(cv::Mat& currentFrame) {
	cv::cvtColor( currentFrame, currentFrame, CV_BGR2GRAY );
	cv::blur( currentFrame, currentFrame, cv::Size(3,3) );
}

/**
 * Searches a given frame for marker bins. Performs more specific filtering and then finds the corners of any bins.
 *
 * @param frame The processed camera frame in which to find the bins.
 * @return  A 2-Dimensional vector containing the corners of each marker. For each marker the corners are listed
 *				in clockwise order starting with the top-left or bottom-right corner when the bin is oriented vertically.
 */
MarkerTarget::Point2DVec MarkerTarget::findBins(cv::Mat& frame) {
	cv::threshold(frame, frame, 170, 255, 1);
	cv::Mat kernel(3, 3, CV_8UC1);
	cv::morphologyEx(frame, frame, cv::MORPH_OPEN, kernel);

	std::vector< std::vector <cv::Point> > contoursPoly;
	frame = getContourMap(frame, contoursPoly);

	Point2DVec corners = getPreliminaryCorners(frame, contoursPoly);
	refineCorners(corners);
	return corners;
}

/**
 * Detects the edges in the given image and uses them to draw a contour and approximate polygons for each contour.
 * Any contour with more than 4 edges and not convex is thrown away in order to find just contours matching the 
 * marker bins.
 *
 * @param frame The current processed camera frame in which to find the contours.
 * @param outContoursPoly An empty 2-Dimenional vector which will be populated with the points belonging to each contour.
 * @return A Mat image with the contours of the detected marker bins.
 */
cv::Mat MarkerTarget::getContourMap(cv::Mat& frame, std::vector< std::vector <cv::Point> >& outContoursPoly) {
	// Detect edges.
	cv::Mat canny_output;
	cv::Canny(frame, canny_output, CANNY_THRESHOLD, CANNY_THRESHOLD * 2, 3 );

	// Find contours
	std::vector<cv::Vec4i> hierarchy;
	std::vector< std::vector <cv::Point> > contours;
	cv::findContours( canny_output, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

	// Approximate each contour as a polygon.
	outContoursPoly.resize(contours.size());
	std::vector<cv::Rect> boundRect( contours.size() );
	for (unsigned int i = 0; i < contours.size(); i++) {
		approxPolyDP( cv::Mat(contours[i]), outContoursPoly[i], 10, true);
		boundRect[i] = cv::boundingRect( cv::Mat(outContoursPoly[i]) );
	}

	// Create output matrix.
	cv::Mat drawing = cv::Mat::zeros( canny_output.size(), CV_8UC1 );
	for (unsigned int i = 0; i < contours.size(); i++) {
		if (isContourConvex(outContoursPoly[i]) && outContoursPoly[i].size() == 4) {
			cv::drawContours( drawing, outContoursPoly, i, 255, 2, 8, hierarchy, 1, cv::Point() );
		}
	}

	return drawing;
}

/**
 * Finds the corners in the given contour and associates each to one of the given contours.
 *
 * @param contourMap A Mat image with only the contours of marker bins drawn on.
 * @param contoursPoly A 2-D vector containing a list of points belonging to each contour.
 * @return  A 2-D vector (std::vector< std::vector< cv::Point2f>>) which is a list of the 4 corners belonging to each detected marker.
 */
MarkerTarget::Point2DVec MarkerTarget::getPreliminaryCorners(const cv::Mat& contourMap,
						const std::vector< std::vector <cv::Point> >& contoursPoly) {
	// Find corners.
	std::vector<cv::Point2f> corners;
	cv::goodFeaturesToTrack(contourMap, corners, 100, 0.4, 25);

	// Associate each corner with a polygon.
	Point2DVec rectangleCorners(contoursPoly.size());
	for (unsigned int i = 0; i < contoursPoly.size(); i++) {
		for (unsigned int k = 0; k < corners.size(); k++) {
			for (unsigned int j = 0; j < contoursPoly[i].size(); j++) {
				int x = corners[k].x, y = corners[k].y;
				int dx = contoursPoly[i][j].x - x; if (dx < 0) dx *= -1;
				int dy = contoursPoly[i][j].y - y; if (dy < 0) dy *= -1;
				if (dx < 5 && dy < 5) {
					rectangleCorners[i].push_back(corners[k]);
					break;
				}
			}
		}
	}

	Point2DVec finalCorners;	// These are the corners we will use to detect the marker image.
	std::vector<cv::RotatedRect> rotatedRects(rectangleCorners.size());
	// Draw the corners.
	for (int i = 0; i < rectangleCorners.size(); i++) {
		if (rectangleCorners[i].size() != 4)
			continue; // Ignore "rectangles" that don't have 4 corners.

		rotatedRects[i] = minAreaRect(rectangleCorners[i]);	 // Draw the minAreaRect to order the corners next to adjacent ones.
		if (!checkRectangleRatio(rotatedRects[i]))
			continue; //Ignore rectangles without the correct ratio.

		cv::Point2f vertices[4]; // Get the vertices of the rectangle.
		rotatedRects[i].points(vertices);

		std::vector<cv::Point2f> tempCorners;
		for (int j = 0; j < 4; j++) {
			std::cout << "[DEBUG] Rectangle Found." << std::endl;
			tempCorners.push_back(vertices[j]);
		}
		finalCorners.push_back(tempCorners); // Add these corners to our final matrix.
	}

	return finalCorners;
}

/**
 * Order the corners so that they are in clockwise order starting with the top-left or bottom-right corner.
 *
 * @param corners An unordered list of corners which will be ordered once the function terminates.
 */
void MarkerTarget::refineCorners(Point2DVec& corners) {
	// Check corners to make sure they are in the correct order.
	for (int i = 0; i < corners.size(); i++) {
		cv::Point2f prevCorner = corners[i][3];
		cv::Point2f curCorner = corners[i][0];
		cv::Point2f nextCorner = corners[i][1];
		double prevLength = sqrt(pow(curCorner.x - prevCorner.x, 2) + pow(curCorner.y - prevCorner.y, 2));
		double nextLength = sqrt(pow(curCorner.x - nextCorner.x, 2) + pow(curCorner.y - nextCorner.y, 2));

		if (nextLength > prevLength) {
			cv::Point2f tempNode = corners[i][0];
			corners[i][0] = corners[i][1];
			corners[i][1] = corners[i][2];
			corners[i][2] = corners[i][3];
			corners[i][3] = tempNode;
		}
	}
}

/**
 * Checks if the ratio of the sides of the marker bin is between 0.4 and 0.6 which defines an accpetable marker bin.
 *
 * @param rectangle The rectangle which sides will will check to see if it is similar to the marker bins.
 */
bool MarkerTarget::checkRectangleRatio(const cv::RotatedRect& rectangle) {
	const double ACCEPTED_RATIO = 0.5;
	const double RATIO_ERROR = 0.1;
	double width = rectangle.size.width;
	double height = rectangle.size.height;
	double ratio = height/width;
	if (width/height < ratio)
		ratio = width/height;

	return (ratio < ACCEPTED_RATIO + RATIO_ERROR && ratio > ACCEPTED_RATIO - RATIO_ERROR);
}

/**
 * Given the corners of the marker bins, detects which marker is inside the bin and gather information such as distance and 
 * angle about the marker. To figure out which marker is present, we compare the marker's image to reference images.
 *
 * @param frame The original camera frame in which to extract the marker image.
 * @param bins A list of corners belonging to each detected marker bin.
 * @return A vector of MarkerDescriptor objects populated with marker type, distances, and angles for each detected marker bin.
 */
std::vector<MarkerTarget::MarkerDescriptor> MarkerTarget::findMarkers(const cv::Mat& frame, const Point2DVec& bins) {
	std::vector<MarkerDescriptor> results;

	for (unsigned int i = 0; i < bins.size(); i++) {
		//Get the minimal image region containing the bin
		cv::Rect bbox = boundingRect(bins[i]);
		cv::Mat markerCandidate = cv::Mat(frame, bbox).clone();

		//Erode it to avoid bright specks in the image
		cv::Mat ellipse = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(3,3), cv::Point(1, 1));
		cv::erode(markerCandidate, markerCandidate, ellipse, cv::Point(1,1), 2);

		cv::adaptiveThreshold(markerCandidate, markerCandidate, 255,
				cv::ADAPTIVE_THRESH_GAUSSIAN_C, cv::THRESH_BINARY, ADAPTIVE_THRESHOLD_KERNEL_SIZE, 0);

		//Shift the bin coordinates so that they match the origin given by the minimal image region
		std::vector<cv::Point2f> sourceCoords;
		cv::Point2f bboxOrigin(bbox.x, bbox.y);
		for(unsigned int j = 0 ; j < bins[i].size() ; j++)
			sourceCoords.push_back(bins[i][j] - bboxOrigin);

		//Compare the candidate against each marker template
		long min = LONG_MAX;
		int minIndex = -1;
		for(unsigned int j = 0 ; j < NUM_REFERENCE_IMGS ; j++) {
			long result = compareImages(sourceCoords, markerCandidate, referenceImgs[j]);
			//Only keep the best result
			if(result < min) {
				minIndex = j;
				min = result;
			}
		}

		//A match was found. Add it to the results.
		if(min < errorTolerance) {

			//FIXME There should be a better way to do this
			ObjectType type = UNDEFINED;
			if(minIndex < 2)
				type = GROUND_TARGET_1;
			else if(minIndex < 4)
				type = GROUND_TARGET_2;
			else if(minIndex < 6)
				type = GROUND_TARGET_3;
			else if(minIndex < 8)
				type = GROUND_TARGET_4;

			results.push_back(MarkerDescriptor(bins[i], type, min));
		}
	}

	return results;
}

/**
 * Compares a candidate marker to each reference image by counting the number of different pixels.
 *
 * @param sourceCoords The coordinates of the candidate markers in cap.
 * @param cap The camera frame in which the marker image is located.
 * @param ref The reference image which we are comparing the similarity of the captured image with.
 * @return The number of different pixels between the candidate and reference images.
 */
long MarkerTarget::compareImages(const std::vector<cv::Point2f> &sourceCoords,
		const cv::Mat& cap, const cv::Mat& ref) {
	/* Obtain a flat version of the image we captured. We do this by creating a matrix which
	 * transforms the image so that the detected corners of the marker end up in the corners
	 * of a new image. */
	std::vector<cv::Point2f> refCoords;
	refCoords.push_back(cv::Point(0,0));
	refCoords.push_back(cv::Point(ref.cols, 0));
	refCoords.push_back(cv::Point(ref.cols, ref.rows));
	refCoords.push_back(cv::Point(0, ref.rows));
	cv::Mat transform = cv::getPerspectiveTransform(sourceCoords, refCoords);
	cv::Mat unwarped = cv::Mat(ref.rows, ref.cols, ref.type());
	cv::warpPerspective(cap, unwarped, transform, ref.size());

	//Apply a binary filter to the transformed image, since its colors might have been distorted
	cv::threshold(unwarped, unwarped, 128, 255, CV_THRESH_BINARY);

	long differences = 0;

	/* Count the differences between the captured marker and the reference marker.
	 * NOTE: There is probably a more efficient way to do this! */
	for(int i = 0 ; i < unwarped.rows ; i++) {
		for(int j = 0 ; j < unwarped.cols ; j++)
			differences += (unwarped.at<int>(i, j) != ref.at<int>(i,j) ? 1 : 0);
	}

	return differences;
}

/**
 * Calculate the distances and angles of the marker from the robot.
 *
 * @param inOutMarker A MarkerDescriptor object containing the location of the bin within the image and will be 
 *			populated with distances and angles.
 */
void MarkerTarget::estimatePose(MarkerDescriptor& inOutMarker) {
	/* We use the solvePnP function, which figures out a transformation
	between a 3D position and 2D image coordinates. We simply assume that
	the marker is at the center of the world. Thus, the solvePnP function
	will give us the transformations to go from the marker to the camera.*/

	//Coordinates of the marker, in centimeters
	std::vector<cv::Point3f> objectCoords;
	objectCoords.push_back(cv::Point3f(-15.24, 30.48, 0));
	objectCoords.push_back(cv::Point3f(15.24, 30.48, 0));
	objectCoords.push_back(cv::Point3f(15.24, -30.48, 0));
	objectCoords.push_back(cv::Point3f(-15.24, -30.48, 0));

	//rvec will contain the rotation transformations, tvec the translations
	cv::Mat rvec, tvec;
	/* The empty vector contains the distortion coefficients. Outside of the
	simulator, this should contain values appropriate for our camera. */
	cv::solvePnP(objectCoords, inOutMarker.imageCorners, intrinsic, std::vector<float>(), rvec, tvec);

	/* The image has y axis vertical, but for us that's the x axis, so we swap x and y. */
	float x = inOutMarker.x_dist = tvec.at<double>(1);
	float y = inOutMarker.y_dist = tvec.at<double>(0);
	float z = inOutMarker.z_dist = tvec.at<double>(2);

	double atanYaw = atan(x / y);
	double degreesYaw = atanYaw * PI / 180.0;
	if(y < 0) {
		if(x < 0) {
			degreesYaw = 180.0 + degreesYaw; // third quadrant
		} else {
			degreesYaw = 180.0 - degreesYaw; //second quadrant
		}
	} else if(x < 0) { //fourth quadrant
		degreesYaw = 360.0 - degreesYaw;
	}

	inOutMarker.yaw_angle = degreesYaw;

	float xyPlaneDistance = sqrt(x * x + y * y);
	double atanPitch = atan(z / xyPlaneDistance);

	/* atanPitch is the angle from the object to the robot. We need the angle
	 * from the robot to the object, so we negate that. */
	inOutMarker.pitch_angle = -(atanPitch * PI / 180.0);
}

int MarkerTarget::add(int x, int y) {
	return x+y;
}