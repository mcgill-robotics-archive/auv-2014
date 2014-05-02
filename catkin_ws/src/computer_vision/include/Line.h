/*
 * LineTarget.h
 *
 *  Created on: Jan 16, 2014
 *      Author: thuy-anh
 */

#ifndef LINE_H
#define LINE_H

#include "VisibleObject.h"

using namespace cv;
using namespace std;

class Line : public VisibleObject {

	double xDistance;
	double yDistance;
	double yaw;
	public:
	Line();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);

	double convertFromPixelsToMetres(int distance, double longSize);
	void applyFilter(cv::Mat& image);
	void thresh_callback(int, void* );
	double relativeYaw(cv::RotatedRect line);
	bool isVisible(cv::RotatedRect line);
	int add(int x, int y);

};

#endif /* LINE_H */
