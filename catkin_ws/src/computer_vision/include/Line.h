/*
 * LineTarget.h
 *
 *  Created on: Jan 16, 2014
 *      Author: thuy-anh
 */

#ifndef LINE_H
#define LINE_H

#include "VisibleObject.h"
#include "CVNode.h"

using namespace cv;
using namespace std;

double camera_focal_length;
double camera_sensor_height;

const int MAX_HSV_HUE = 179; //In OpenCV, hue can take 180 values (0 to 179)
const int MAX_HSV_SAT = 255;
const int MAX_HSV_VALUE = 255;

class Line : public VisibleObject {

	double xDistance;
	double yDistance;
	double zDistance;
	double yaw;

	public:
	Line(const CVNode& _parent, CVNode::LineParameters& _params);
	~Line();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

	double convertFromPixelsToMetres(int distance, double longSize);
	void applyFilter(cv::Mat& image);
	// Line::thresh_callback(int, void* );
	double relativeYaw(cv::RotatedRect line);
	bool isVisible(cv::RotatedRect line);
	int add(int x, int y);
	double calculateApproxDistanceToLine (double yaw, double sizeLong, double sizeWide, cv::Mat& currentFrame);

	private:
	CVNode::LineParameters params;
};

#endif /* LINE_H */
