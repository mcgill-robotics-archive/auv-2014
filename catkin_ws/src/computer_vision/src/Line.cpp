/*
 * Line.cpp
 *
 *  Created on: Jan 16, 2014
 *      Author: thuy-anh
 Author: Usman Ehtesham Gul
 Author: Joel Lat
 */
#include "Line.h"

int kernelSize = 3;
Mat src;
Mat src_gray;
cv::Mat filteredImage;
int thresh = 300;
int max_thresh = 500;
RNG rng(12345);

const double PI = std::atan(1.0) * 4;
int smallestAreaTemp = 10000;
double movingAverage[5];
int pos = 0;
int reset = 0;

double calculateSize(double x1, double x2, double y1, double y2);
cv::RotatedRect thresh_callback(int, void*);
//double calculateApproxDistanceToLine (double yaw, double sizeLong, double sizeWide, cv::Mat& currentFrame);
//double convertFromPixelsToMetres(int distance, double longSize);
bool visibility = false;

Line::Line(const CVNode& _parent, CVNode::LineParameters& _params) : VisibleObject(_parent), params(_params) {
	xDistance = 0.0;
	yDistance = 0.0;
	zDistance = 0.0;
	//yaw = 0.0;
}

std::vector<computer_vision::VisibleObjectData*> Line::retrieveObjectData(
		cv::Mat& currentFrame) {
	bool isVisible;
	double yawAngle = -90.0;
	double pitchAngle = 90.0;
	//double xDistance = 1;
	//double yDistance = 3;
	//double zDistance = 0;
	std::vector<computer_vision::VisibleObjectData*> messagesToReturn;

	// Creates a pointer that will point to a computer_vision::VisibleObjectData object.
	computer_vision::VisibleObjectData* visibleObjectData =
			new computer_vision::VisibleObjectData();

	// Apply filters on the cv::Mat object.
	applyFilter(currentFrame);

	// Check if door is visible

	if (visibility) {
		// Get object data
		// [...]

		// Return gathered data to caller
		visibleObjectData->object_type = visibleObjectData->LANE;
		visibleObjectData->pitch_angle = 0.0;
		visibleObjectData->yaw_angle = yaw;
		visibleObjectData->x_distance = xDistance;
		visibleObjectData->y_distance = yDistance;
		visibleObjectData->z_distance = zDistance;

		messagesToReturn.push_back(visibleObjectData);
		visibility = false;
		return (messagesToReturn);
	} else {
		return (messagesToReturn);
	}
}
/**
 * Applies filter so we can detect the line
 * @param image Current frame given from video
 */

void Line::applyFilter(cv::Mat& image) {
	cv::Mat imageHSV;
	Point2f vertices[4];
	src = image;
	cv::cvtColor(image, imageHSV, CV_BGR2HSV);
	cv::GaussianBlur(imageHSV, imageHSV, cv::Size(kernelSize, kernelSize), 0, 0);

	cv::RotatedRect line;
	cv::RotatedRect holderTemp;

	//Range for the simulator
	cv::inRange(imageHSV, cv::Scalar(5, 100, 100), cv::Scalar(15, 255, 255),
			filteredImage);
	holderTemp = thresh_callback(0, 0);
	line = holderTemp;

	//namedWindow("Filtered Image", CV_WINDOW_NORMAL);
	//resizeWindow("Filtered Image", 1018, 715);
	//imshow("Filtered Image", filteredImage);

	// HSV (0-180, 0-255, 0-255)
	// Scalar (H, S, V)

	//Range for the line in down video
	//##########################################################################
	// Filter 1
	//##########################################################################
	cv::inRange(imageHSV, cv::Scalar(params.hue_range1_begin, params.sat_range1_begin, params.value_range1_begin), cv::Scalar(params.hue_range1_end, params.sat_range1_end, params.value_range1_end),
			filteredImage);
	//namedWindow( "Filtered Image", CV_WINDOW_NORMAL );
	//resizeWindow("Filtered Image", 1018, 715);
	//imshow( "Filtered Image", filteredImage );
	holderTemp = thresh_callback(0, 0);
	if (holderTemp.size.area() >= line.size.area()) {
		line = holderTemp;
	}

	//Range for the down video
	//##########################################################################
	// Filter 2
	//##########################################################################
	cv::inRange(imageHSV, cv::Scalar(params.hue_range2_begin, params.sat_range2_begin, params.value_range2_begin), cv::Scalar(params.hue_range2_end, params.sat_range2_end, params.value_range2_end),
			filteredImage);
	holderTemp = thresh_callback(0, 0);
	if (holderTemp.size.area() >= line.size.area()) {
		line = holderTemp;
	}
	//line = holderTemp;

	//Range for the run2-cropped
	//##########################################################################
	// Filter 3
	//##########################################################################
	cv::inRange(imageHSV, cv::Scalar(params.hue_range3_begin, params.sat_range3_begin, params.value_range3_begin), cv::Scalar(params.hue_range3_end, params.sat_range3_end, params.value_range3_end),
			filteredImage);
	holderTemp = thresh_callback(0, 0);
	if (holderTemp.size.area() >= line.size.area()) {
		line = holderTemp;
	}

	//Range for the rosbag videos
	//##########################################################################
	// Filter 4
	//##########################################################################
	cv::inRange(imageHSV, cv::Scalar(params.hue_range4_begin, params.sat_range4_begin, params.value_range4_begin), cv::Scalar(params.hue_range4_end, params.sat_range4_end, params.value_range4_end),
			filteredImage);
	holderTemp = thresh_callback(0, 0);
	if (holderTemp.size.area() >= line.size.area()) {
		line = holderTemp;
	}

	Mat drawing = Mat::zeros(filteredImage.size(), CV_8UC3);

	//draws located line
	line.points(vertices);

	double sizeLong;
	double sizeWide;

	sizeLong = calculateSize(vertices[0].x, vertices[(0 + 1) % 4].x,
			vertices[0].y, vertices[(0 + 1) % 4].y);
	sizeWide = calculateSize(vertices[0].x, vertices[(0 - 1) % 4].x,
			vertices[0].y, vertices[(0 - 1) % 4].y);

	if (sizeLong < sizeWide) {
		double temp;
		temp = sizeLong;
		sizeLong = sizeWide;
		sizeWide = temp;
	}

	int centerX = line.center.x;
	int centerY = line.center.y;
	double yDistancePixels = 0;
	double xDistancePixels = 0;

	cv::line(drawing, Point(centerX, centerY), Point(centerX, centerY),
			cv::Scalar(0, 0, 255), 2, 8, 0);
	cv::line(drawing,
			Point(drawing.size().width / 2, drawing.size().height / 2),
			Point(drawing.size().width / 2, drawing.size().height / 2),
			cv::Scalar(255, 0, 0), 2, 8, 0);

	visibility = isVisible(line);
	//for (int i = 0; i < 4; ++i){
	//cv::line(drawing, vertices[i], vertices[(i + 1) % 4], cv::Scalar(0, 255, 0), 1, CV_AA);
	//}
	yaw = relativeYaw(line);

	//Keep the five last yaw recordings and if the yaw is too far from the average, do not take it
	//Set visibility constant to false
	if (pos < 5) {
		movingAverage[pos % 5] = yaw;
		pos++;
	} else {
		double sum = 0;
		for (int i = 0; i < 5; i++) {
			sum += movingAverage[i];
		}
		if (std::abs(yaw - sum / 5) < 0.40) {
			movingAverage[pos % 5] = yaw;
			pos++;
		} else {
			visibility = false;
		}
	}

	if (!visibility)
		reset++;
	else
		reset = 0;

	if (reset > 10) {
		reset = 0;
		for (int x = 0; x < 5; x++)
			movingAverage[x] = 0;
		pos = 0;
	}
	//If output is opposite, then invert
	//xDistance = drawing.size().width/2 - centerX;
	//yDistance = drawing.size().height/2 - centerY;
	//isVisible(line);
	yDistancePixels = centerX - drawing.size().width / 2;
	xDistancePixels = drawing.size().height / 2 - centerY;
	yDistance = convertFromPixelsToMetres(yDistancePixels, sizeLong);
	xDistance = convertFromPixelsToMetres(xDistancePixels, sizeLong);

	//Calculate approximative distance to line assuming the line is right in front of the camera
	double approxDistance = calculateApproxDistanceToLine(yaw, sizeWide,
			sizeLong, image);

	//Calculate better approximation to determine real distance to line
	//double yOffset, xOffset;
	//yOffset = convertFromPixelsToMetres(std::abs((drawing.size().height/2 )- centerX),sizeLong);
	//xOffset = convertFromPixelsToMetres(std::abs((drawing.size().width/2 )- centerY),sizeLong);

	//double distance2D = std::sqrt((std::pow (xOffset, 2)) + (std::pow (yOffset, 2)));

	//Determines the zDistance
	zDistance = approxDistance;

	//if a line is visible draw the rectangle
	if (visibility) {
		for (int i = 0; i < 4; ++i) {
			cv::line(drawing, vertices[i], vertices[(i + 1) % 4],
					cv::Scalar(0, 255, 0), 1, CV_AA);
		}
	}

	/// Show in a window
	if (parent.get_down_using_helpers()) {
		namedWindow("Contours", CV_WINDOW_NORMAL);
		imshow("Contours", drawing);
	}

	/// Create Window
	//char* source_window = "Source";

	//namedWindow(source_window, CV_WINDOW_NORMAL);
	//resizeWindow(source_window, 1018, 715);
	//imshow(source_window, src);

}

/**
 * Record all rectangles and selects ones that have a bigger area than 500 pixels
 */
cv::RotatedRect thresh_callback(int, void*) {
	Mat canny_output;
	vector<vector<Point> > contours;
	vector<Vec4i> hierarchy;
	cv::Mat img = cv::Mat::zeros(500, 500, CV_8UC3);

	/// Detect edges using canny
	Canny(filteredImage, canny_output, thresh, thresh * 2, 3);
	/// Find contours
	findContours(canny_output, contours, hierarchy, CV_RETR_TREE,
			CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

	/// Draw contours
	Mat drawing = Mat::zeros(canny_output.size(), CV_8UC3);

	Point2f vertices[4];
	cv::RotatedRect line;
	std::vector<std::vector<cv::Point> > contours_poly(contours.size());

	//evaluates all found lines and returns largest line line that fits ratio
	for (int i = 0; i < contours.size(); i++) {
		approxPolyDP(cv::Mat(contours[i]), contours_poly[i], 4, true);
		std::vector<cv::Point> hull;
		cv::convexHull(cv::Mat(contours_poly[i]), hull);
		cv::Mat hull_points(hull);
		cv::RotatedRect current = minAreaRect(hull_points);

		//if(current.size.area()<350){
		//continue;
		// }

		if (current.size.area() < 500) {
			continue;
		}

		current.points(vertices);
		if (current.size.area() > line.size.area()) {
			line = minAreaRect(hull_points);
		}
	}
	return (line);
}
/**
 * Determines distance of robot from line
 * @param line Line found using contours
 * @return Distance of roboto from line
 */
double distance(cv::RotatedRect line) {
	return 0.;
}

double calculateSize(double x1, double x2, double y1, double y2) {
	//	double xCoordSquared;
	//	double yCoordSquared;
	//	double sizeLong;
	//	xCoordSquared = std::pow ((x1 - x2), 2);
	//	yCoordSquared = std::pow ((y1 - y2), 2);
	//	sizeLong = std::sqrt (xCoordSquared + yCoordSquared);
	//	return sizeLong;
	return std::sqrt(std::pow((x1 - x2), 2) + std::pow((y1 - y2), 2));
}

double Line::convertFromPixelsToMetres(int distance, double longSize) {
	//double distanceMetres;
	//distanceMetres = (distance * 1.2)/ longSize;
	//return distanceMetres;
	return (distance * 1.2) / longSize;
}

/**
 * Determines Whether there is currently a visible line
 * @param line Line found using contours
 * @return Whether line is visible or not
 */
bool Line::isVisible(cv::RotatedRect line) {
	//if (line.size.height != 0 || line.size.width !=0)
	//	return true;
	//return false;
	return (line.size.height != 0 || line.size.width != 0);
}

/**
 * Evaluates found line and returns relative yaw between the sub and the line
 * @param line Line found using contours
 * @return Relative yaw between the sub and the line
 */
double Line::relativeYaw(cv::RotatedRect line) {
	double angle = line.angle;
	//re-adjust incorrect angles
	if (line.size.height < line.size.width)
		angle = angle + 90;

	//cout << "Relative Yaw Is " << angle << endl;
	//cout<< " -----------------------------------------------" << endl;
	angle = (angle * PI) / 180;
	return angle;
}

int Line::add(int x, int y) {
	return x + y;
}

double Line::calculateApproxDistanceToLine(double yaw, double sizeLong,
		double sizeWide, cv::Mat& currentFrame) {
	double realLineHeight;
	double imageLineHeight;
	double approxDistance;

	realLineHeight = (std::cos(yaw) * 1.2) + (std::sin(yaw) * 0.15);
	imageLineHeight = (std::cos(yaw) * sizeLong) + (std::sin(yaw) * sizeWide);

	//cout << currentFrame.size().height << "\n";
	//cout << camera_focal_length;


	approxDistance = (parent.get_camera_focal_length() * realLineHeight
			* currentFrame.size().height) / (imageLineHeight
			* parent.get_camera_sensor_height());

	return approxDistance;
}

