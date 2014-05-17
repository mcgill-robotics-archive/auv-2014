/**
 * This object is used to detect the buoy object.
 *
 * @author Aleksi Sapon
 */

#include "Buoy.h"

Buoy::Buoy(const CVNode& _parent) : VisibleObject(_parent) {
	cv::namedWindow("Filter", CV_WINDOW_KEEPRATIO);
	cv::namedWindow("Threshold", CV_WINDOW_KEEPRATIO);
	cv::namedWindow("Drawing", CV_WINDOW_KEEPRATIO);
}

Buoy::~Buoy() {

}

/**
 * Applies the necessary filters to the current frame, checks if the buoy
 * object is present and, if it is, extracts the necessary information.
 * @param currentFrame Current camera frame
 * @return If the buoy is present, it returns a pointer to an ObjectData
 *  which contains the information gathered on the buoy. If the buoy is
 *  not present in the current frame, it returns the zero pointer (NULL).
 */
std::vector<computer_vision::VisibleObjectData*> Buoy::retrieveObjectData(cv::Mat& currentFrame) {
	bool isVisible;
	double yawAngle = -90.0;
	double pitchAngle = 90.0;
	double xDistance = -1.0;
	double yDistance = -1.0;
	double zDistance = -1.0;
	std::cout << "Looking for buoys..." << std::endl;
	// Creates a pointer that will point to a computer_vision::VisibleObjectData object.
	std::vector<computer_vision::VisibleObjectData*> detectedObjects;
	computer_vision::VisibleObjectData* visibleObjectData = new computer_vision::VisibleObjectData();

	// Apply filters on the cv::Mat object.
	std::vector<DetectedBuoy> detected = detectBuoys(&currentFrame, RED_BUOY);
	
	
	if (!detected.empty()) {
		std::cout << "Buoy found." << std::endl;
		xDistance = detected[0].center.x;
		yDistance = detected[0].center.y;
		// FIXME: detected[0].center is a 2D point and has no z attribute.
		// zDistance = detected[0].center.z;
	
		// Return gathered data to caller
		visibleObjectData->object_type = visibleObjectData->BUOY;
		visibleObjectData->pitch_angle = yawAngle;
		visibleObjectData->yaw_angle = pitchAngle;
		visibleObjectData->x_distance = xDistance;
		visibleObjectData->y_distance = yDistance;
		visibleObjectData->z_distance = zDistance;
		detectedObjects.push_back(visibleObjectData);
	} 
	std::cout << "Detected Objects: " << detectedObjects.size() << std::endl;
	return detectedObjects;
}

void Buoy::applyFilter(cv::Mat& currentFrame) {

};

/** 
 * Computes some values for color correction
 */
void Buoy::initColorCorrection() {
	// Get the gray scale equivalent of the color sample by averaging, this is the correct color
	int whiteAverage = (colorSample[0] + colorSample[1] + colorSample[2]) / 3;
	// Create a scalar for the correct color
	cv::Scalar balancedWhite(whiteAverage, whiteAverage, whiteAverage);
	// The amount of color correction to apply is the difference between the sample color and the correct color,
	// minus the difference between the target color and the correct color
	colorCorrection = colorSample - balancedWhite - (targetColor - balancedWhite);
}

/**
 * Clamps the integer between low and high so that low <= n <= high
 */
int Buoy::clamp(int n, int low, int high) {
	n = n > high ? high: n;
	return n < low ? low : n;
}

/**
 * Applies color correction to the color
 */
void Buoy::colorCorrect(uchar* b, uchar* g, uchar* r) {
	// Calculate the difference between the color and the sample color
	float bDiff = colorSample[0] - *b;
	float gDiff = colorSample[1] - *g;
	float rDiff = colorSample[2] - *r;
	// Calculate a weight, the closer the color is the sample, the closer the weight is to 1,
	// falling off linearly to 0 when nearing 0 or 255
	float bWeight = 1 - (bDiff > 0 ? bDiff / colorSample[0] : -bDiff / (255 - colorSample[0]));
	float gWeight = 1 - (gDiff > 0 ? gDiff / colorSample[1] : -gDiff / (255 - colorSample[1]));
	float rWeight = 1 - (rDiff > 0 ? rDiff / colorSample[2] : -rDiff / (255 - colorSample[2]));
	// Apply the color correction, weighting it
	*b = clamp(*b - colorCorrection[0] * bWeight, 0, 255);
	*g = clamp(*g - colorCorrection[1] * gWeight, 0, 255);
	*r = clamp(*r - colorCorrection[2] * rWeight, 0, 255);
}

/**
 * Applies the color correction to the image
 */
void Buoy::applyColorCorrection(cv::Mat* image) {
	// Info for image size
	int rows = image->rows;
	int cols = image->cols * image->channels();
	// Iterate over entire image, applying the color correction 
	uchar* row;
	for (int i = 0; i < rows; i++) {
		row = image->ptr<uchar>(i);
		for (int j = 0; j < cols; j += 3) {
			colorCorrect(&row[j], &row[j + 1], &row[j + 2]);
		}
	}
}

/** 
 * An thresholding function that supports negative hue values (important for red)
 */
void Buoy::inRangeWrapped(cv::Mat* source, cv::Scalar low, cv::Scalar high, cv::Mat* destination) {
	// Info for image size
	int srcRows = source->rows;
	int srcCols = source->cols * source->channels();
	int dstRows = destination->rows;
	int dstCols = destination->cols * destination->channels();
	// Ensure parameters are correct
	assert(srcCols == dstCols * 3);
	assert(srcRows == dstRows);
	// Ensure the hue range is between 0 and 180
	assert(low[0] <= high[0]);
	int lowH = (int(low[0]) + 180) % 180;
	int highH = (int(high[0]) + 180) % 180;
	// Iterate over entire image 
	uchar* srcRow;
	uchar* dstRow;
	for (int i = 0; i < srcRows; i++) {
		srcRow = source->ptr<uchar>(i);
		dstRow = destination->ptr<uchar>(i);
		for (int j = 0; j < srcCols; j += 3) {
			int h = int(srcRow[j]);
			int s = int(srcRow[j + 1]);
			int v = int(srcRow[j + 2]);
			bool hPass;
			if (lowH < highH) {
				hPass = h >= lowH && h <= highH;
			} else {
				hPass = (h >= lowH && h <= 180) || (h >= 0 && h <= highH);
			}
			if (hPass && s >= low[1] && s <= high[1] && v >= low[2] && v <= high[2]) {
				dstRow[j / 3] = 255;
			} else {
				dstRow[j / 3] = 0;
			}
		}
	}
}


/** 
 * Gets the black and white thresholded image from the original color image
 */
void Buoy::getThresholdImage(cv::Mat* image, BuoyColorProfile buoy, cv::Mat* destination) {
	// Smooth to reduce noise
	cv::GaussianBlur(*image, *image, cv::Size(3, 3), 0);
	// Convert from RGB to HSV
	cv::Mat HSVImage(image->rows, image->cols, CV_8UC3);
	cv::cvtColor(*image, HSVImage, CV_BGR2HSV);
	// Generate the threshold image
	inRangeWrapped(&HSVImage, buoy.lowColor, buoy.highColor, destination);
}

/** 
 * Returns the number of samples to get the desired coverage (as a percentage) of the area of the disk of the provided radius
 */
int Buoy::getSampleCount(float radius, float coverage) {
	return coverage * M_PI * radius * radius;
}

/** 
 * Uses a Monte-Carlo method to check is the disk is at least as dense (in the sense of no missing pieces) as the desired density (as a percentage)
 *  It's used to discard invalid circles formed by random contours 
 */
bool Buoy::checkDisk(cv::Mat* image, cv::Point* center, int radius, int sampleCount, float minDensity) {
	// Initialize the random number generator and ranges for the full disk in polar coordinates
	static std::mt19937 generator;
	static std::uniform_real_distribution<float> angleRange(0, 2 * M_PI);
	std::uniform_real_distribution<float> radiusRange(0, radius);
	// Total density of the samples
	float density = 0;
	// Do the sampling
 	for (int i = 0; i < sampleCount; i++) {
 		// Pick a random point on the disk
 		float angle = angleRange(generator);
 		float radius = radiusRange(generator);
 		int xx = int(radius * cos(angle) + center->x);
 		int yy = int(radius * sin(angle) + center->y);
 		// Add the normalized weight
		density += float(image->at<uchar>(yy, xx)) / 255;
 	}
 	// Normalize the total density
 	density /= sampleCount;
 	return density >= minDensity;
}

/** 
 * Returns the approximate distance from the buoy in meters based its radius in the image, and the height of the said image
 */
float Buoy::getDistance(float radius, int imageHeight) {
	return (BUOY_FOCAL_LENGTH * BUOY_RADIUS * imageHeight) / (radius * BUOY_SENSOR_HEIGHT) / 1000;
}

/** 
 * The main method of the detection algorithm, use RED_BUOY or YELLOW_BUOY for color profiles
 */
std::vector<DetectedBuoy> Buoy::detectBuoys(cv::Mat* image, BuoyColorProfile buoy) {
	// Compute color correction data from the color sample and target color once
	static bool isColorCorrectionComputed = false;
	if (!isColorCorrectionComputed) {
		initColorCorrection();
		isColorCorrectionComputed = true;
	}
	// Apply color correction
	applyColorCorrection(image);
	cv::imshow("Filter", *image);
	// Threshold it
	cv::Mat thresholdImage(image->rows, image->cols, CV_8UC1);
	getThresholdImage(image, buoy, &thresholdImage);	
	cv::imshow("Threshold", thresholdImage);
	// Blur the image an do circle detection
	cv::GaussianBlur(thresholdImage, thresholdImage, cv::Size(25, 25), 0);	
	std::vector<cv::Vec3f> circles;
	cv::HoughCircles(thresholdImage, circles, CV_HOUGH_GRADIENT, 2, thresholdImage.rows / 8, 50, 100);
	std::cout << "Circles: " << circles.size() << std::endl;

	// Draw detected circles.
	cv::Mat drawing(image->rows, image->cols, CV_8UC1);
	for (int i = 0; i < circles.size(); i++) {
		cv::circle(drawing, cv::Point(circles[i][0], circles[i][1]), circles[i][2], cv::Scalar(255, 0, 0));
		cv::imshow("Drawing", drawing);
	}

	// Process the circles
	std::vector<DetectedBuoy> buoys;
	for (int i = 0; i < circles.size(); i++) {
  		cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
 		int radius = cvRound(circles[i][2]);
 		// Check that we have a well defined disk
 		if (checkDisk(&thresholdImage, &center, radius, getSampleCount(radius, 0.15f), 0.85f)) {
 			DetectedBuoy detected(center, radius);
 			// Calculate distance
 			detected.distance = getDistance(radius, image->rows);
 			buoys.push_back(detected);
 		}
 	}
 	std::cout << "Buoys: " << buoys.size() << std::endl;
 	return buoys;
}
