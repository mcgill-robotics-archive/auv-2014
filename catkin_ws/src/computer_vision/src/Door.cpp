/**
 * This object is used to detect the gate object.
 *
 * @author Jean-Sebastien Dery
 * @author Renaud Dagenais
 * @author Haris Haidary
 */

#include "Door.h"

#define USE_CV_WINDOWS 1

const std::string COLOR_THRESH_WINDOW = "color_thresh_window";
const std::string TRACKBARS_WINDOW = "trackbars_window";

/**
 * Constructor.
 */
Door::Door() {
	ROS_INFO("%s", "The Gate object has been created.");

#ifdef USE_CV_WINDOWS
	ROS_INFO("%s",
			"Creating the window that will be displaying the frame after the HSV threshold is applied.");
	cv::namedWindow(COLOR_THRESH_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::namedWindow(TRACKBARS_WINDOW, CV_WINDOW_KEEPRATIO);

	cv::createTrackbar("end_hsv_hue_threshold", TRACKBARS_WINDOW, &end_hsv_hue_threshold, MAX_HSV_VALUE);
	cv::createTrackbar("start_hsv_value_threshold", TRACKBARS_WINDOW, &start_hsv_value_threshold, MAX_HSV_VALUE);
	cv::createTrackbar("end_hsv_value_threshold", TRACKBARS_WINDOW, &end_hsv_value_threshold, MAX_HSV_VALUE);
	cv::createTrackbar("gate_ratio_error", TRACKBARS_WINDOW, &gate_ratio_error, MAX_HSV_VALUE);
	cv::createTrackbar("max_number_points", TRACKBARS_WINDOW, &max_number_points, MAX_HSV_VALUE);
	cv::createTrackbar("polygon_approximation_threshold", TRACKBARS_WINDOW, &polygon_approximation_threshold, MAX_HSV_VALUE);
#endif
}

/**
 * Destructor.
 */
Door::~Door() {
#ifdef USE_CV_WINDOWS
	cv::destroyWindow(COLOR_THRESH_WINDOW);
#endif
}

/**
 * Applies the necessary filters to the current frame, checks if the door
 * object is present and, if it is, extracts the necessary information.
 * @param currentFrame Current camera frame
 * @return If the door is present, it returns a pointer to an ObjectData
 *  which contains the information gathered on the door. If the door is
 *  not present in the current frame, it returns the zero pointer (NULL).
 */
std::vector<computer_vision::VisibleObjectData*> Door::retrieveObjectData(cv::Mat& currentFrame) {
	bool isVisible;
	double yawAngle = -90.0;
	double pitchAngle = 90.0;
	double xDistance = 1;
	double yDistance = 3;
	double zDistance = 4;
	std::vector<computer_vision::VisibleObjectData*> messagesToReturn;

	// Creates a pointer that will point to a computer_vision::VisibleObjectData object.
	computer_vision::VisibleObjectData* visibleObjectData;

	// Apply filters on the cv::Mat object.
	applyFilter(currentFrame);

	ROS_INFO("%s",
			"The processing for the Gate object is completed. Now creating the ROS message.");

	// Check if door is visible
	isVisible = true;

	if (isVisible) {
		// Get object data
		// [...]

		// Return gathered data to caller
		visibleObjectData->object_type = visibleObjectData->DOOR;
		visibleObjectData->pitch_angle = yawAngle;
		visibleObjectData->yaw_angle = pitchAngle;
		visibleObjectData->x_distance = xDistance;
		visibleObjectData->y_distance = yDistance;
		visibleObjectData->z_distance = zDistance;

		messagesToReturn.push_back(visibleObjectData);

		return (messagesToReturn);
	} else {
		return (messagesToReturn);
	}
}

/**
 * Function that will apply filter on the image so we can detect the door.
 *
 * @param currentFrame The frame to which we need to apply the filters.
 */
void Door::applyFilter(cv::Mat& currentFrame) {
	HSV_ENDING_FILTER_RANGE = cv::Scalar(end_hsv_hue_threshold, 255, end_hsv_value_threshold);
	HSV_STARTING_FILTER_RANGE = cv::Scalar(0, 0, start_hsv_value_threshold);

	std::vector<cv::RotatedRect> potentialMatchRectangles;

	// Ok so what I need to do is accept more rectangles and apply logic to filter them
	// no bounds for the maximum of points, and only filter for a percentage of the heigh relative to the image resolution.
	// also, the rectangles should be almost vertically aligned.

	ROS_INFO("%s", "Applying filter to the current frame.");

	// Converts the current frame to HSV in order to ease color filtering (with varying brightness).
	cv::Mat currentFrameInHSV = convertFromBGRXToHSV(currentFrame);

	// Apply a Gaussian Blur filter to regularise the pixels from the camera image.
	// This is done in a try to reduce the noise generated by the sensor of the camera.
	cv::GaussianBlur(currentFrameInHSV, currentFrameInHSV, cv::Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0);

	// Finds all the contours in the image and store them in a vector containing vectors of points.
	std::vector<std::vector<cv::Point> > detectedContours = findContoursFromHSVFrame(currentFrameInHSV);

	// Goes through all the identified shapes for a first filtering.
	for (int rectangleID = 0; rectangleID < detectedContours.size(); rectangleID++) {
		// Gets the contour to analyse in this loop iteration.
		std::vector<cv::Point> contourToAnalyse = detectedContours.at(rectangleID);

//		// Approximates a polygonal curve(s) with the specified precision on the contourToAnalyse vector.
//		std::vector<cv::Point> contourAfterPolygonApproximation;
//		approxPolyDP(contourToAnalyse, contourAfterPolygonApproximation, polygon_approximation_threshold, true);

		// Gets the number of points that define this contour.
		int numberOfPointsInContour = contourToAnalyse.size();

		// Displays only the contours that have a number of points in the specified range and respect the ratio.
		std::cout << "[DEBUG] Number of points in this contour= " << numberOfPointsInContour << std::endl;

		// Finds a rotated rectangle that defines the contour of the object.
		cv::RotatedRect foundRectangle = cv::minAreaRect(cv::Mat(contourToAnalyse));

		// Determining the Width, Height and Ratio of the rotated rectangle.
		float width = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.width : foundRectangle.size.height;
		float height = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.height : foundRectangle.size.width;
		float heightWidthRatio = std::abs(height / width - GATE_RATIO);
		float angle = foundRectangle.angle;

		// The angle of the rectangle will always be between [-90:0).
		if (numberOfPointsInContour >= MIN_NUMBER_POINTS && (angle <= -80 || angle >= -10)) {

			// Add the rectangle that is a potential match for the orange cylinders of the gate.
			potentialMatchRectangles.push_back(foundRectangle);

			cv::Point2f vertices[4];
			foundRectangle.points(vertices);

			// Goes through each vertex and connects them to write the edges of the rectangle.
			for (int i = 0; i < 4; ++i) {
				std::cout << "[DEBUG] (" << i << ") = (" << vertices[i].x << ";" << vertices[i].y << ")" << std::endl;
				cv::line(currentFrame, vertices[i], vertices[(i + 1) % 4], BLUE_BGRX, 1, CV_AA);
			}
			drawPointsOfContour(currentFrame, contourToAnalyse, GREEN_BGRX);

			// Write the text information right next to the rectangle.
			float approximateDistanceWithObject = (FOCAL_LENGTH * DOOR_REAL_HEIGHT * currentFrame.size().height) / (height * CAMERA_SENSOR_HEIGHT);
			// Draws text containing the dimensions on each rectangles.
			std::stringstream ssWidth;
			std::stringstream ssHeight;
			std::stringstream ssDistance;
			std::stringstream ssAngle;
			ssWidth << "Width=" << width;
			ssHeight << "Height=" << height;
			ssDistance << "Distance=" << approximateDistanceWithObject << " mm";
			ssAngle << "Angle=" << angle;
			putText(currentFrame, (ssWidth).str(),
					cv::Point(vertices[0].x, vertices[0].y),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
			putText(currentFrame, (ssHeight).str(),
					cv::Point(vertices[0].x, vertices[0].y + 10),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
			putText(currentFrame, (ssDistance).str(),
					cv::Point(vertices[0].x, vertices[0].y + 20),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
			putText(currentFrame, (ssAngle).str(),
					cv::Point(vertices[0].x, vertices[0].y + 30),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
		} else {
			drawPointsOfContour(currentFrame, contourToAnalyse, RED_BGRX);
		}
	}

	int numberOfPotentialMatch = potentialMatchRectangles.size();
	//  Here I assume that the two rectangles that I have are the orange cylinders of the gate.
	if (numberOfPotentialMatch == 2) {
		cv::RotatedRect rectangleOne = potentialMatchRectangles.at(0);
		cv::RotatedRect rectangleTwo = potentialMatchRectangles.at(1);

		cv::Point2f centerOfRectangleOne = rectangleOne.center;
		cv::Point2f centerOfRectangleTwo = rectangleTwo.center;

		float centerX = (centerOfRectangleOne.x + centerOfRectangleTwo.x)/2;
		float centerY = (centerOfRectangleOne.y + centerOfRectangleTwo.y)/2;

		cv::Point centerPoint;
		centerPoint.x = centerX;
		centerPoint.y = centerY;
		cv::circle(currentFrame, centerPoint, 30, WHITE_BGRX, 2, 5);

		cv::Point centerPoint2;
		centerPoint2.x = currentFrame.cols;
		centerPoint2.y = currentFrame.rows;
		cv::circle(currentFrame, centerPoint2, 10, MAUVE_BGRX, 2, 5);

//		ROS_INFO("%s", ("Width of image=" << currentFrame.cols << " Height of image=" << currentFrame.rows));

		cv::line(currentFrame, centerOfRectangleOne, centerOfRectangleTwo, WHITE_BGRX, 1, CV_AA);
	}

//	// Goes through the potential matches of the rectangles for a second filtering.
//	for (int i = 0; i < potentialMathRectangleIDs.size(); i++) {
//		int rectangleID = potentialMathRectangleIDs.at(i);
//
//
//	}
}

/**
 * Finds all contours (based on a HSV range) from an HSV frame and returns the cloud of points determining the contour.
 *
 * @param frameInHSV The frame in HSV color space.
 * @return The std::vector of std::vector of points containing the clouds of all contours in the image.
 */
std::vector<std::vector<cv::Point> > Door::findContoursFromHSVFrame(const cv::Mat& frameInHSV) {
	// Creates the Mat object that will contain the filtered image (inRange HSV).
	cv::Mat inRangeHSVFrame;
	// Generates a new Mat object that only contains a certain range of HSV values.
	// Don't forget that we are not using BGRX, but the HSV color space.
	cv::inRange(frameInHSV, HSV_STARTING_FILTER_RANGE, HSV_ENDING_FILTER_RANGE, inRangeHSVFrame);

#ifdef USE_CV_WINDOWS
	cv::imshow(COLOR_THRESH_WINDOW, inRangeHSVFrame);
#endif

	// Finds the contours in the images.
	cv::Mat inRangeFrame = inRangeHSVFrame.clone();
	// So this vector will contain vectors of points that will form shapes in the image.
	std::vector<std::vector<cv::Point> > detectedContours;
	cv::findContours(inRangeHSVFrame, detectedContours, CV_RETR_CCOMP,
			CV_CHAIN_APPROX_SIMPLE);

	return (detectedContours);
}

/**
 * Draws the points defining a contour.
 *
 * @param frame The cv::Mat object on which the points will be drawn.
 * @param contour The vector containing the points to be drawn.
 */
void Door::drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour, cv::Scalar COLOR) {
	// Draw each single point that forms the polygon.
	for (int j = 0; j < contour.size(); j++) {
		cv::Point singlePoint = contour.at(j);
		rectangle(frame, singlePoint, singlePoint, COLOR, 8, 8);
	}
}
