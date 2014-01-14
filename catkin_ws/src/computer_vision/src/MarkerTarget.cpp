/*
 * MarkerTarget.cpp
 *
 *  Created on: Nov 20, 2013
 *      Author: fred
 */

#include <stdlib.h>
#include <opencv2/imgproc/imgproc.hpp>
#include "MarkerTarget.h"

MarkerTarget::MarkerTarget() {
	//Load the reference images
	for(int i = 0 ; i < NUM_REFERENCE_IMGS; i += 2) {
		//Load the image, add it to the list, mirror it on both axes and add it again
	}

	/* To set the error tolerance, assume that all reference images
	 * have the same size. */
	errorTolerance = (int) (((float) (referenceImgs[0].cols * referenceImgs[0].rows)) * 0.15f);
}

computer_vision::VisibleObjectData* MarkerTarget::retrieveObjectData(cv::Mat& currentFrame) {
	applyFilter(currentFrame);
	cv::Mat filteredFrame = currentFrame.clone();

	Point2DVec binCorners = findBins(currentFrame);
	std::vector<MarkerDescriptor> markers = findMarkers(filteredFrame, binCorners);

	//Estimate 3D position of each found marker.

	return NULL;
}

void MarkerTarget::applyFilter(cv::Mat& currentFrame) {
	cv::cvtColor( currentFrame, currentFrame, CV_BGR2GRAY );
	cv::blur( currentFrame, currentFrame, cv::Size(3,3) );
}

MarkerTarget::Point2DVec MarkerTarget::findBins(cv::Mat& frame) {
	cv::threshold(frame, frame, 170, 255, 1);
	cv::Mat kernel(3, 3, CV_8UC1);
	cv::morphologyEx(frame, frame, cv::MORPH_OPEN, kernel);

	Point2DVec contoursPoly;
	cv::Mat contourMap = getContourMap(frame, contoursPoly);

	Point2DVec corners = getPreliminaryCorners(contourMap, contoursPoly);
	refineCorners(corners);
	return corners;
}

cv::Mat MarkerTarget::getContourMap(cv::Mat& frame, Point2DVec& outContoursPoly) {
	// Detect edges.
	cv::Mat canny_output;
	cv::Canny(frame, canny_output, CANNY_THRESHOLD, CANNY_THRESHOLD * 2, 3 );

	// Find contours
	std::vector<cv::Vec4i> hierarchy;
	Point2DVec contours;
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

MarkerTarget::Point2DVec MarkerTarget::getPreliminaryCorners(const cv::Mat& contourMap,
						const Point2DVec& contoursPoly) {
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
			tempCorners.push_back(vertices[j]);
		}
		finalCorners.push_back(tempCorners); // Add these corners to our final matrix.
	}

	return finalCorners;
}

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
			long result = compareCandidateWithReference(sourceCoords, markerCandidate, referenceImgs[j]);
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

long MarkerTarget::compareCandidateWithReference(const std::vector<cv::Point2f> &sourceCoords,
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
