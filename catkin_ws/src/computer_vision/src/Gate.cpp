/**
 * This object is used to detect the gate object.
 *
 * @author Jean-Sebastien Dery
 * @author Renaud Dagenais
 * @author Haris Haidary
 * @author Frederic Lafrance
 */

#include "Gate.h"

#define USE_CV_WINDOWS 1

const std::string COLOR_THRESH_WINDOW = "color_thresh_window";
const std::string TRACKBARS_WINDOW = "trackbars_window";

float centimetersPerPixel;

cv::Point centerOfCurrentFrame;

/**
 * Constructor.
 */
Gate::Gate() {

#ifdef USE_CV_WINDOWS
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
Gate::~Gate() {
#ifdef USE_CV_WINDOWS
	cv::destroyWindow(COLOR_THRESH_WINDOW);
	cv::destroyWindow(TRACKBARS_WINDOW);
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
std::vector<computer_vision::VisibleObjectData*> Gate::retrieveObjectData(cv::Mat& currentFrame) {
	std::vector<computer_vision::VisibleObjectData*> messagesToReturn;
	m_isVisible = false;

	ROS_INFO("%s", "About to process the image in order to detect the gate object.");

	applyFilter(currentFrame);

	ROS_INFO("%s", "The processing for the Gate object is completed. Now creating the ROS message.");

	if (m_isVisible) {
		computer_vision::VisibleObjectData* visibleObjectData = new computer_vision::VisibleObjectData();		

		// Return gathered data to caller
		visibleObjectData->object_type = visibleObjectData->DOOR;
		visibleObjectData->yaw_angle = m_yawAngle;
		visibleObjectData->pitch_angle = 0.0; //Pitch angle is not used anymore
		visibleObjectData->x_distance = m_xDistance;
		visibleObjectData->y_distance = m_yDistance;
		visibleObjectData->z_distance = m_zDistance;

		messagesToReturn.push_back(visibleObjectData);

		ROS_INFO("%s", "Added a ROS message in the list of messages to return to the cv node.");
	}

	ROS_INFO("%s", ("About to return the ROS message list to the cv node that contains " + boost::lexical_cast<std::string>(messagesToReturn.size()) + " element(s).").c_str());

	return (messagesToReturn);
}

/**
 * Function that will apply filter on the image so we can detect the door.
 *
 * @param currentFrame The frame to which we need to apply the filters.
 */
void Gate::applyFilter(cv::Mat& currentFrame) {
	HSV_ENDING_FILTER_RANGE = cv::Scalar(end_hsv_hue_threshold, 255, end_hsv_value_threshold);
	HSV_STARTING_FILTER_RANGE = cv::Scalar(0, 0, start_hsv_value_threshold);

	std::vector<PoleCandidate> potentialMatchRectangles;

	// Ok so what I need to do is accept more rectangles and apply logic to filter them
	// no bounds for the maximum of points, and only filter for a percentage of the heigh relative to the image resolution.
	// also, the rectangles should be almost vertically aligned.

	//ROS_INFO("%s", "Applying filter to the current frame.");

	centerOfCurrentFrame.x = currentFrame.cols/2;
	centerOfCurrentFrame.y = currentFrame.rows/2;
	cv::circle(currentFrame, centerOfCurrentFrame, 5, MAUVE_BGRX, 2, 5);
	//ROS_INFO("%s", ("Width of image=" + boost::lexical_cast<std::string>(currentFrame.cols) + " pixels. Height of image="  + boost::lexical_cast<std::string>(currentFrame.rows) + " pixels.").c_str());

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

		// Gets the number of points that define this contour.
		int numberOfPointsInContour = contourToAnalyse.size();

		// Finds a rotated rectangle that defines the contour of the object.
		cv::RotatedRect foundRectangle = cv::minAreaRect(cv::Mat(contourToAnalyse));

		// Determining the Width, Height and Ratio of the rotated rectangle.
		// Here by convention the height is the longest side of the rectangle.
		float width = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.width : foundRectangle.size.height;
		float height = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.height : foundRectangle.size.width;
		float heightWidthRatio = height / width;

		/* Calculate the angle of the rectangle. It is in interval [0, 180) and
		works as expected (i.e. rectangle on its side has angle 0, rectangle upright has
		angle 90) */
		cv::Point2f vertices[4];
		foundRectangle.points(vertices);
		cv::Point2f p = vertices[0], q = vertices[1];
		cv::Point2f diff = p - q;
		float distanceFirstPoints = sqrt(diff.x * diff.x + diff.y * diff.y);
		//Check whether the two points we have form a width or a height.
		if(std::abs(distanceFirstPoints - width) < std::abs(distanceFirstPoints - height)) {
			p = q;
			q = vertices[2];
		}

		float opp = std::abs(p.y - q.y); //Want positive opposite side value
		//Get signed adjacent value (it's the highest point minus the lowest point)
		float adj = p.x - q.x;
		if(q.y < p.y)
			adj = -adj;

		float rectangleAngle = atan(opp / adj) * 180.0 / 3.141592654;
		if(rectangleAngle < 0)
			rectangleAngle = 180 + rectangleAngle;

		float angleError = std::abs(rectangleAngle - DESIRED_ANGLE_OF_RECT);

		// First, make sure that our ratio is acceptably close to what we expect
		// Also, we need at least 4 points in our contour.
		if (	heightWidthRatio > MIN_GATE_RATIO && 
			numberOfPointsInContour >= 4 &&
			angleError < ANGLE_RECT_ERROR	) {

			float approximateDistanceWithObject = (FOCAL_LENGTH * DOOR_REAL_HEIGHT * currentFrame.size().height) / 
							(height * CAMERA_SENSOR_HEIGHT);

			/* We correct this distance, because it is only valid if the object is close to the center of the screen */
			
			//This is experimental.
			/*float offsetRatio = std::abs(centerOfCurrentFrame.x - foundRectangle.center.x) / centerOfCurrentFrame.x;
			approximateDistanceWithObject /= (1.0 + 0.06 * offsetRatio);
			approximateDistanceWithObject *= (1.0 + 0.06 * offsetRatio);*/


			//Take into account the y-axis difference.
			float mPerPxAtObject = DOOR_REAL_HEIGHT / height;
			float yMOffset = std::abs(centerOfCurrentFrame.x - foundRectangle.center.x) * mPerPxAtObject;
			float objectAngle = atan(yMOffset / approximateDistanceWithObject);
			float correctedDistance = approximateDistanceWithObject / cos(objectAngle);

			// Add the rectangle that is a potential match for the orange cylinders of the gate.
			PoleCandidate found = {height, width, rectangleAngle, objectAngle, correctedDistance, foundRectangle.center};
			potentialMatchRectangles.push_back(found);


			//Draw text on the image
			drawPointsOfContour(currentFrame, contourToAnalyse, GREEN_BGRX);
			putText(currentFrame, "Distance=" + boost::lexical_cast<std::string>(correctedDistance),
					cv::Point(vertices[0].x, vertices[0].y + 30),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
			putText(currentFrame, "ObjectAngle=" + boost::lexical_cast<std::string>(objectAngle * 180.0 / 3.141592654),
					cv::Point(vertices[0].x, vertices[0].y + 40),
					cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
					CV_AA);
			
		} else {
			drawPointsOfContour(currentFrame, contourToAnalyse, RED_BGRX);
		}

		// Goes through each vertex and connects them to write the edges of the rectangle.
		for (int i = 0; i < 4; ++i) {
			cv::line(currentFrame, vertices[i], vertices[(i + 1) % 4], BLUE_BGRX, 1, CV_AA);
		}

		putText(currentFrame, "Width=" + boost::lexical_cast<std::string>(width),
				cv::Point(vertices[0].x, vertices[0].y),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
				CV_AA);
		putText(currentFrame, "Height=" + boost::lexical_cast<std::string>(height),
				cv::Point(vertices[0].x, vertices[0].y + 10),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
				CV_AA);
		putText(currentFrame, "Angle=" + boost::lexical_cast<std::string>(rectangleAngle),
				cv::Point(vertices[0].x, vertices[0].y + 20),
				cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
				CV_AA);

	}

	int numberOfPotentialMatch = potentialMatchRectangles.size();
	//  Here I assume that the two rectangles that I have are the orange cylinders of the gate.
	if (numberOfPotentialMatch == 2) {
		m_isVisible = true;
		PoleCandidate p1 = potentialMatchRectangles[0];
		PoleCandidate p2 = potentialMatchRectangles[1];

		PoleCandidate& closest = (p1.dist < p2.dist) ? p1 : p2;
		PoleCandidate& farthest = (p1.dist < p2.dist) ? p2 : p1;

		const float CLOSEST_SQR = closest.dist * closest.dist;
		const float FARTHEST_SQR = farthest.dist * farthest.dist;
		const float GATE_SQR = GATE_WIDTH * GATE_WIDTH;
		float cosFar = (FARTHEST_SQR + GATE_SQR - CLOSEST_SQR) / (2.0 * farthest.dist * GATE_WIDTH);

		// 90 - angleFar- farthest.objectAngle
		 m_yawAngle = (3.141592654 / 2.0) - acos(cosFar) - farthest.objectAngleRad;

		if(closest.center.x < farthest.center.x)
			m_yawAngle = -m_yawAngle;

		//Get the x,y coordinates of the gate in the world
		float x1 = cos(p1.objectAngleRad) * p1.dist, x2 = cos(p2.objectAngleRad) * p2.dist;
		float y1 = sin(p1.objectAngleRad) * p1.dist, y2 = sin(p2.objectAngleRad) * p2.dist;

		if(p1.center.x - centerOfCurrentFrame.x < 0)
			y1 = -y1;

		if(p2.center.x - centerOfCurrentFrame.x < 0)
			y2 = -y2;

		m_xDistance = (x1 + x2) / 2.0;
		m_yDistance = (y1 + y2) / 2.0;

		int centerY = (p1.center.y + p2.center.y) / 2;
		float avgMPerPx = ((DOOR_REAL_HEIGHT / p1.h) + (DOOR_REAL_HEIGHT / p2.h)) / 2.0;
		m_zDistance = (centerY - centerOfCurrentFrame.y) * avgMPerPx;

		cv::Point centerPoint((p1.center.x + p2.center.x) / 2, centerY);
		//Draw the point on-screen
		cv::circle(currentFrame, centerPoint, 30, GREEN_BGRX, 2, 5);
		cv::line(currentFrame, p1.center, p2.center, WHITE_BGRX, 1, CV_AA); 

		std::cout << "[DEBUG] Gate found: <" << m_xDistance << "," << m_yDistance << "," << m_zDistance << "> yaw " << 
				m_yawAngle * 180.0 / 3.141592654 << std::endl;
	}
}

/**
 * Finds all contours (based on a HSV range) from an HSV frame and returns the cloud of points determining the contour.
 *
 * @param frameInHSV The frame in HSV color space.
 * @return The std::vector of std::vector of points containing the clouds of all contours in the image.
 */
std::vector<std::vector<cv::Point> > Gate::findContoursFromHSVFrame(const cv::Mat& frameInHSV) {

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
void Gate::drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour, cv::Scalar COLOR) {
	// Draw each single point that forms the polygon.
	for (int j = 0; j < contour.size(); j++) {
		cv::Point singlePoint = contour.at(j);
		rectangle(frame, singlePoint, singlePoint, COLOR, 8, 8);
	}
}
