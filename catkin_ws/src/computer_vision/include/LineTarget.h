/*
 * LineTarget.h
 *
 *  Created on: Jan 16, 2014
 *      Author: thuy-anh
 */

#ifndef LINETARGET_H_
#define LINETARGET_H_

#include "VisibleObject.h"


using namespace cv;
using namespace std;

class LineTarget : public VisibleObject {
	double xDistance;
	double yDistance;
	double yaw;
	public:
	LineTarget();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);


	void applyFilter(cv::Mat& image);
	void thresh_callback(int, void* );
	double relativeYaw(cv::RotatedRect line);
	bool isVisible(cv::RotatedRect line);

};



#endif /* LINETARGET_H_ */
