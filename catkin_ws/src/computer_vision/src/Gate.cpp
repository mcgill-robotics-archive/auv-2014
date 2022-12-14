/**
 * This object is used to detect the gate object.
 *
 * @author Jean-Sebastien Dery
 * @author Frederic Lafrance
 * @author Renaud Dagenais
 * @author Haris Haidary
 */

#include "Gate.h"

float centimetersPerPixel;

cv::Point centerOfCurrentFrame;

const std::string COLOR_THRESH_WINDOW = "color_thresh_window";
const std::string TRACKBARS_WINDOW = "trackbars_window";

/**
 * Constructor.
 */
Gate::Gate(const CVNode& _parent, CVNode::GateParameters& _params) : VisibleObject(_parent), params(_params) {

	ROS_INFO("%s", (std::string(__PRETTY_FUNCTION__) + ":: a Gate object was instantiated.").c_str());

	if (parent.get_front_using_helpers()) {
		cv::namedWindow(COLOR_THRESH_WINDOW, CV_WINDOW_KEEPRATIO);
		cv::namedWindow(TRACKBARS_WINDOW, CV_WINDOW_KEEPRATIO);

		cv::createTrackbar("Hue range 1 begin", TRACKBARS_WINDOW, &(params.hue_range1_begin), MAX_HSV_HUE);
		cv::createTrackbar("Hue range 1 end", TRACKBARS_WINDOW, &(params.hue_range1_end), MAX_HSV_HUE);
		cv::createTrackbar("Hue range 2 begin", TRACKBARS_WINDOW, &(params.hue_range2_begin), MAX_HSV_HUE);
		cv::createTrackbar("Hue range 2 end", TRACKBARS_WINDOW, &(params.hue_range2_end), MAX_HSV_HUE);
		cv::createTrackbar("Value begin", TRACKBARS_WINDOW, &(params.value_range_begin), MAX_HSV_VALUE);
		cv::createTrackbar("Value end", TRACKBARS_WINDOW, &(params.value_range_begin), MAX_HSV_VALUE);
	}
}

/**
 * Destructor.
 */
Gate::~Gate() {
	if (parent.get_front_using_helpers()) {
		cv::destroyWindow(COLOR_THRESH_WINDOW);
		cv::destroyWindow(TRACKBARS_WINDOW);
	}
}

/**
 * Applies the necessary filters to the current frame, checks if the door
 * object is present and, if it is, extracts the necessary information.
 * @param currentFrame Current camera frame
 * @return If the door is present, it returns a pointer to an ObjectData
 *  which contains the information gathered on the door. If the door is
 *  not present in thel current frame, it returns the zero pointer (NULL).
 */
std::vector<computer_vision::VisibleObjectData*> Gate::retrieveObjectData(cv::Mat& currentFrame) {
	std::vector<computer_vision::VisibleObjectData*> messagesToReturn;
	m_isVisible = false;

	std::vector<cv::Mat> channels;
	cv::split(currentFrame, channels);
	cv::normalize(channels[0], channels[0], 0, 192, cv::NORM_MINMAX);
	cv::normalize(channels[1], channels[1], 0, 224, cv::NORM_MINMAX);
	cv::normalize(channels[2], channels[2], 0, 255, cv::NORM_MINMAX);

	//cv::Scalar meanBlue = cv::mean(channels[0]);
	//cv::subtract(channels[0], meanBlue * 0.25, channels[0]);
	cv::merge(channels, currentFrame);	

	applyFilter(currentFrame);

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

//		ROS_INFO("%s", "Added a ROS message in the list of messages to return to the cv node.");
	}

//	ROS_INFO("%s", ("About to return the ROS message list to the cv node that contains " + boost::lexical_cast<std::string>(messagesToReturn.size()) + " element(s).").c_str());

	return (messagesToReturn);
}	

/**
 * Function that will apply filter on the image so we can detect the door.
 *
 * @param currentFrame The frame to which we need to apply the filters.
 */
void Gate::applyFilter(cv::Mat& currentFrame) {
	std::vector<PoleCandidate> potentialMatchRectangles;
	centerOfCurrentFrame.x = currentFrame.cols/2;
	centerOfCurrentFrame.y = currentFrame.rows/2;

	// Converts the current frame to HSV in order to ease color filtering (with varying brightness).
	cv::Mat currentFrameInHSV = convertFromBGRXToHSV(currentFrame);

	// Apply a Gaussian Blur filter to regularise the pixels from the camera image.
	// This is done in a try to reduce the noise generated by the sensor of the camera.
	cv::GaussianBlur(currentFrameInHSV, currentFrameInHSV, cv::Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0);

	// Finds all the contours in the image and store them in a vector containing vectors of points.
	std::vector<std::vector<cv::Point> > detectedContours = findContoursFromHSVFrame(currentFrameInHSV);

	// Goes through all the identified shapes for a first filtering.
	for (int rectangleID = 0; rectangleID < detectedContours.size(); rectangleID++) {

		// Finds a rotated rectangle that defines the contour of the object.
		std::vector<cv::Point> contour = detectedContours.at(rectangleID);
		PoleCandidate pole = findRectangleForContour(contour);

		bool passesFilter = false;
		// Filter the rectangles based on several parameters
		if ((pole.h / pole.w) > params.min_pole_ratio && // Low ratios look like a square, not a pole
			contour.size() >= params.min_number_points_contour && // Contours with too few points are most likely not right
			std::abs(pole.rectangleAngleDeg - params.pole_desired_angle_deg) < params.pole_angle_error_deg && // Angle error
			cv::contourArea(contour) / (double) (pole.h * pole.w) > params.min_convexity_ratio // Reject non-convex contours
			) {
				passesFilter = true;
				computePolarCoordinates(pole, centerOfCurrentFrame, currentFrame.size().height);
				potentialMatchRectangles.push_back(pole);

				//Draw contour points
				drawPointsOfContour(currentFrame, contour, GREEN_BGRX);
		} else {
			drawPointsOfContour(currentFrame, contour, RED_BGRX);
		}

		if (passesFilter)
			writePoleCandidateInfo(pole, currentFrame);
	}

	int numberOfPotentialMatch = potentialMatchRectangles.size();

	if (numberOfPotentialMatch > 1) {
		//Note: This is a very expensive way of checking all possible combinations!		
		int ip1, ip2;
		for(int i = 0 ; i < numberOfPotentialMatch - 1 && !m_isVisible ; i++) {
			PoleCandidate& p1 = potentialMatchRectangles[i];

			for(int j = i + 1 ; j < numberOfPotentialMatch && !m_isVisible ; j++) {
				PoleCandidate& p2 = potentialMatchRectangles[j];
				m_isVisible = handleTwoVisiblePoles(p1, p2, centerOfCurrentFrame);
				if(m_isVisible) {
					ip1 = i;
					ip2 = j;
				}
			}
		}

		if(m_isVisible) {
			PoleCandidate& p1 = potentialMatchRectangles[ip1];
			PoleCandidate& p2 = potentialMatchRectangles[ip2];
			drawGateInfo(currentFrame, p1, p2);
		}	
	}
	
	//Center of image circle
	cv::circle(currentFrame, centerOfCurrentFrame, 5, MAUVE_BGRX, 2, 5);
}

bool Gate::handleTwoVisiblePoles(PoleCandidate& p1, PoleCandidate& p2, cv::Point centerOfCurrentFrame) {
	
	PoleCandidate& closest = (p1.dist < p2.dist) ? p1 : p2;
	PoleCandidate& farthest = (p1.dist < p2.dist) ? p2 : p1;

	//If both poles are on the same side of the image, the robot angle is the largest minus the smallest
	float sumAngles = 0.f;
	if( std::max(p1.center.x, p2.center.x) < centerOfCurrentFrame.x ||
	    std::min(p1.center.x, p2.center.x) > centerOfCurrentFrame.x )
		sumAngles = std::max(p1.objectAngleRad, p2.objectAngleRad) -
				std::min(p1.objectAngleRad, p2.objectAngleRad);
	else //Otherwise it's the normal sum
		sumAngles = p1.objectAngleRad + p2.objectAngleRad;


	/* Use law of sines to compute far angle */
	float sinFar = sin(sumAngles) * closest.dist / params.gate_width_m;

	// 90 - angleFar- farthest.objectAngle
	m_yawAngle = (3.141592654 / 2.0) - asin(sinFar) - farthest.objectAngleRad;

	if(closest.center.x < farthest.center.x)
		m_yawAngle = -m_yawAngle;

	//Get the x,y coordinates of the gate in the world
	float x1 = cos(p1.objectAngleRad) * p1.dist, x2 = cos(p2.objectAngleRad) * p2.dist;
	float y1 = sin(p1.objectAngleRad) * p1.dist, y2 = sin(p2.objectAngleRad) * p2.dist;

	if(p1.center.x - centerOfCurrentFrame.x < 0)
		y1 = -y1;

	if(p2.center.x - centerOfCurrentFrame.x < 0)
		y2 = -y2;

	//If the error of the gate's width is too large, reject it.
	float dx = x2 - x1, dy = y2 - y1;
	float detectedWidthSqr = dx * dx + dy * dy;
	float widthErr = std::abs((params.gate_width_m * params.gate_width_m) - detectedWidthSqr);
	if(widthErr > params.gate_width_error_mSqr) //TODO Parametrize this
		return false;

	m_xDistance = (x1 + x2) / 2.0;
	m_yDistance = (y1 + y2) / 2.0;

	int centerY = (p1.center.y + p2.center.y) / 2;
	float avgMPerPx = ((params.gate_height_m / p1.h) + (params.gate_height_m / p2.h)) / 2.0;
	m_zDistance = (centerY - centerOfCurrentFrame.y) * avgMPerPx;

	return true;
}

/**
 * Based on a contour, return a partially filled pole candidate structure that contains information about the rectangle
 * bounding the contour.
 *
 * @param contour A contour of points in the image
 * @return A pole candidate. The objectAngleRad and dist fields are NOT valid.
 */
Gate::PoleCandidate Gate::findRectangleForContour(std::vector<cv::Point>& contour) {
	PoleCandidate ret;	


	cv::RotatedRect boundingRect = cv::minAreaRect(cv::Mat(contour));
	// Determining the Width, Height and Ratio of the rotated rectangle.
	// Here by convention the height is the longest side of the rectangle.
	ret.w = (boundingRect.size.width < boundingRect.size.height) ? 
			boundingRect.size.width : boundingRect.size.height;
	ret.h = (boundingRect.size.width < boundingRect.size.height) ? 
			boundingRect.size.height : boundingRect.size.width;

	ret.center = boundingRect.center;

	/* Calculate the angle of the rectangle. It is in interval [0, 180) and
	works as expected (i.e. rectangle on its side has angle 0, rectangle upright has
	angle 90) */
	cv::Point2f vertices[4];
	boundingRect.points(vertices);
	cv::Point2f p = vertices[0], q = vertices[1];
	cv::Point2f diff = p - q;
	float distanceFirstPoints = sqrt(diff.x * diff.x + diff.y * diff.y);
	//Check whether the two points we have form a width or a height.
	if(std::abs(distanceFirstPoints - ret.w) < std::abs(distanceFirstPoints - ret.h)) {
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

	ret.rectangleAngleDeg = rectangleAngle;
	return ret;
}

/**
 * Based on a pole candidate, calculate its distance and angle in the world.
 */
void Gate::computePolarCoordinates(PoleCandidate& pole, cv::Point frameCenter, float frameHeight) {
	/* First approximation using the canonical formula */
	float approximateDistanceWithObject = (parent.get_camera_focal_length() * params.gate_height_m * frameHeight) / 
							(pole.h * parent.get_camera_sensor_height());

	/* We correct this distance, because it is only valid if the object is close to the center of the screen */
	float mPerPxAtObject = params.gate_height_m / pole.h;
	float yMOffset = std::abs(frameCenter.x - pole.center.x) * mPerPxAtObject;
	pole.objectAngleRad = atan(yMOffset / approximateDistanceWithObject);
	pole.dist = approximateDistanceWithObject / cos(pole.objectAngleRad);
}

/**
 * Finds all contours (based on a HSV range) from an HSV frame and returns the cloud of points determining the contour.
 *
 * @param frameInHSV The frame in HSV color space.
 * @return The std::vector of std::vector of points containing the clouds of all contours in the image.
 */
std::vector<std::vector<cv::Point> > Gate::findContoursFromHSVFrame(const cv::Mat& frameInHSV) {
	cv::Scalar end(params.hue_range1_end, 255, params.value_range_end);
	cv::Scalar start(params.hue_range1_begin, 0, params.value_range_begin);


	//cv::Mat inRangeHSVFrame;
	// Creates the Mat object that will contain the filtered image (inRange HSV).
	cv::Mat inRangeHSVFrame(frameInHSV.rows, frameInHSV.cols, CV_8UC1);
	// Generates a new Mat object that only contains a certain range of HSV values.
	// Don't forget that we are not using BGRX, but the HSV color space.

	uchar* src = frameInHSV.data;
	uchar* dst = inRangeHSVFrame.data;
	int lim = frameInHSV.total();
	for(int i = 0 ; i < lim * 3 ; i += 3, dst++) {
		
		// (Hue in 1st range OR Hue in 2nd range) AND Value in range
		if( ((src[i] >= params.hue_range1_begin && src[i] <= params.hue_range1_end) ||
			(src[i] >= params.hue_range2_begin && src[i] <= params.hue_range2_end)) &&
			(src[i + 2] >= params.value_range_begin && src[i + 2] <= params.value_range_end) ) {
			*dst = 255;
		} else {
			*dst = 0;
		}
	}

	cv::dilate(inRangeHSVFrame, inRangeHSVFrame, cv::Mat(), cv::Point(-1, -1), 4);

	if (parent.get_front_using_helpers()) {
		cv::imshow(COLOR_THRESH_WINDOW, inRangeHSVFrame);
	}

	// Finds the contours in the images.
	cv::Mat inRangeFrame = inRangeHSVFrame.clone();
	// So this vector will contain vectors of points that will form shapes in the image.
	std::vector<std::vector<cv::Point> > detectedContours;
	cv::findContours(inRangeHSVFrame, detectedContours, CV_RETR_CCOMP,
			CV_CHAIN_APPROX_SIMPLE);

	return (detectedContours);
}

/**
 * Draw information about a pole candidate on the frame.
 *
 */
void Gate::writePoleCandidateInfo(PoleCandidate& pole, cv::Mat& currentFrame) {


	putText(currentFrame, "Distance=" + boost::lexical_cast<std::string>(pole.dist),
		cv::Point(pole.center.x, pole.center.y + 30),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
		CV_AA);
	putText(currentFrame, "ObjectAngle=" + boost::lexical_cast<std::string>(pole.objectAngleRad * 180.0 / 3.141592654),
		cv::Point(pole.center.x, pole.center.y + 40),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
		CV_AA);
	

	putText(currentFrame, "Width=" + boost::lexical_cast<std::string>(pole.w),
		cv::Point(pole.center.x, pole.center.y),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
		CV_AA);
	putText(currentFrame, "Height=" + boost::lexical_cast<std::string>(pole.h),
		cv::Point(pole.center.x, pole.center.y + 10),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
		CV_AA);
	putText(currentFrame, "RectangleAngle=" + boost::lexical_cast<std::string>(pole.rectangleAngleDeg),
		cv::Point(pole.center.x, pole.center.y + 20),
		cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
		CV_AA);

	
}

void Gate::drawGateInfo(cv::Mat& frame, PoleCandidate& p1, PoleCandidate& p2) {

	cv::Point centerPoint((p1.center.x + p2.center.x) / 2, (p1.center.y + p2.center.y) / 2);
	//Draw the point on-screen
	cv::circle(frame, centerPoint, 30, GREEN_BGRX, 2, 5);
	cv::line(frame, p1.center, p2.center, WHITE_BGRX, 1, CV_AA);

	putText(frame, "Distance <x,y,z>=" + boost::lexical_cast<std::string>(m_xDistance) + " " + 
			boost::lexical_cast<std::string>(m_yDistance) + " " + 
			boost::lexical_cast<std::string>(m_zDistance),
			cv::Point(centerPoint.x, centerPoint.y + 35),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
			CV_AA);

	putText(frame, "Yaw=" + boost::lexical_cast<std::string>((180.0/3.141592654) * m_yawAngle),
			cv::Point(centerPoint.x, centerPoint.y + 45),
			cv::FONT_HERSHEY_COMPLEX_SMALL, 0.4, WHITE_BGRX, 1,
			CV_AA);
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
		rectangle(frame, singlePoint, singlePoint, COLOR, 2, 2);
	}
}
