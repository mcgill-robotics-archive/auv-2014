#include "opencv2/highgui/highgui.hpp"
#include "opencv/cv.h"
#include "iostream"
#include "cmath"

// Defines the different constants used:
const int CANNY_RATIO = 3;
const int CANNY_LOW_THRESHOLD = 200;
const int KERNEL_SIZE = 3;
const int GATE_RATIO = 16; // It's 1:16 for the width and height.
const int GATE_RATIO_ERROR = 5;
const float FOCAL_LENGTH = 8;
const float DOOR_REAL_HEIGHT = 1219.2;
const float CAMERA_SENSOR_HEIGHT = 6.26;
const int ESCAPE_KEY = 27;
const int TIME_BETWEEN_FRAME = 150; // This is in miliseconds.
const int MIN_NUMBER_EDGES = 4;
const int MAX_NUMBER_EDGES = 50;

// Defines the basic colors used in the BGRX color space.
const cv::Scalar GREEN_BGRX = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE_BGRX = cv::Scalar(255, 0, 0);
const cv::Scalar RED_BGRX = cv::Scalar(0, 0, 255);
const cv::Scalar WHITE_BGRX = cv::Scalar(255, 255, 255);
const cv::Scalar MAUVE_BGRX = cv::Scalar(212, 115, 212);
const cv::Scalar HSV_STARTING_FILTER_RANGE = cv::Scalar(0, 0, 40);
const cv::Scalar HSV_ENDING_FILTER_RANGE = cv::Scalar(20, 255, 220);

// Defines the different windows used.
const std::string ORIGINAL_WINDOW = "ORIGINAL_WINDOW";
const std::string FILTERED_WINDOW = "FILTERED_WINDOW";

// Defines the functions used in this cpp file.
void executeVideo(cv::VideoCapture videoCapture);
void applyFilter(cv::Mat& currentFrame);
cv::Mat convertFromBGRXToHSV(const cv::Mat& currentFrame);
std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV);
void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour);

/**
 * Goes through each frames of the video and loops back to the begining on the last frame.
 *
 * @param videoCapture The cv::VideoCapture object that contain the video's stream.
*/
void executeVideo(cv::VideoCapture videoCapture) {
	while (1) {
		cv::Mat currentFrame;

		bool isCurrentFrameNotEmpty = videoCapture.read(currentFrame);
		if (!isCurrentFrameNotEmpty) {
			std::cout << "[INFO] Looping back to the first frame." << std::endl;
			videoCapture.set(CV_CAP_PROP_POS_FRAMES, 35);
			videoCapture.read(currentFrame);
		}

		applyFilter(currentFrame);

		// Manages the frequency at which the frames are being updated. And closes the windows whenever ESC is pressed.
		if (cv::waitKey(TIME_BETWEEN_FRAME) == ESCAPE_KEY) {
			std::cout << "[WARNING] The user has pressed ESC. Terminating the process" << std::endl;
			exit(EXIT_SUCCESS);
		}

		// FIXME: This is only so we can have a visual support of what's going on. It will not be in the final code.
		cv::imshow(ORIGINAL_WINDOW, currentFrame);
	}
}

/**
 * Converts the cv::Mat object passed in parameter and converts it from BGRX to HSV color space.
 *
 * @param currentFrame The cv::Mat object to be returned.
 * @return The converted cv::Mat object in HSV color space.
 */
cv::Mat convertFromBGRXToHSV(const cv::Mat& currentFrame) {
	cv::Mat currentFrameInHSV;
	cv::cvtColor(currentFrame, currentFrameInHSV, CV_BGR2HSV);
	return (currentFrameInHSV);
}

/**
 * Finds all contours (based on a HSV range) from an HSV frame and returns the cloud of points determining the contour.
 *
 * @param frameInHSV The frame in HSV color space.
 * @return The std::vector of std::vector of points containing the clouds of all contours in the image.
 */
std::vector<std::vector<cv::Point> > findContoursFromHSVFrame(const cv::Mat& frameInHSV) {
	// Creates the Mat object that will contain the filtered image (inRange HSV).
	cv::Mat inRangeHSVFrame;
	// Generates a new Mat object that only contains a certain range of HSV values.
	// Don't forget that we are not using BGRX, but the HSV color space.
	cv::inRange(frameInHSV, HSV_STARTING_FILTER_RANGE, HSV_ENDING_FILTER_RANGE, inRangeHSVFrame);

	// Finds the contours in the images.
	cv::Mat inRangeFrame = inRangeHSVFrame.clone();
	// So this vector will contain vectors of points that will form shapes in the image.
	std::vector<std::vector<cv::Point> > detectedContours;
	cv::findContours(inRangeHSVFrame, detectedContours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	// FIXME: This is only so we can have a visual support of what's going on. It will not be in the final code.
	cv::imshow(FILTERED_WINDOW, inRangeFrame);

	return (detectedContours);
}

/**
 * Function that will apply filter on the image so we can detect the door.
 *
 * @param currentFrame The frame to which we need to apply the filters.
*/
void applyFilter(cv::Mat& currentFrame) {
	std::cout << "[INFO] Applying filter to the current frame." << std::endl;

	// Converts the current frame to HSV in order to ease color filtering (with varying brightness).
	cv::Mat currentFrameInHSV = convertFromBGRXToHSV(currentFrame);

	// Apply a Gaussian Blur filter to regularise the pixels from the camera image.
	// This is done in a try to reduce the noise generated by the sensor of the camera.
	cv::GaussianBlur(currentFrameInHSV, currentFrameInHSV, cv::Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0);

	// Finds all the contours in the image and store them in a vector containing vectors of points.
	std::vector<std::vector<cv::Point> > detectedContours = findContoursFromHSVFrame(currentFrameInHSV);

	for (int i = 0 ; i < detectedContours.size() ; i++) {
		// Gets the contour to analyse in this loop iteration.
		std::vector<cv::Point> contourToAnalyse = detectedContours.at(i);
		// Approximates a polygonal curve(s) with the specified precision on the contourToAnalyse vector.
		std::vector<cv::Point> contourAfterPolygonApproximation;
		approxPolyDP(contourToAnalyse, contourAfterPolygonApproximation, 3, true);

		// Gets the number of points that define this contour.
		int numberOfPointsInContour = contourAfterPolygonApproximation.size();

		// Displays only the contours that have a number of points in the specified range and respect the ratio.
		if (numberOfPointsInContour >= MIN_NUMBER_EDGES && numberOfPointsInContour <= MAX_NUMBER_EDGES) {
			std::cout << "[DEBUG] Number of points in this contour= " << numberOfPointsInContour << std::endl;

			// Finds a rotated rectangle that defines the contour of the object.
			cv::RotatedRect foundRectangle = cv::minAreaRect(cv::Mat(contourAfterPolygonApproximation));

			// Determining the Width, Height and Ratio of the rotated rectangle.
			float width = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.width : foundRectangle.size.height;
			float height = (foundRectangle.size.width < foundRectangle.size.height) ? foundRectangle.size.height : foundRectangle.size.width;
			float heightWidthRatio = std::abs(height/width - GATE_RATIO);

			if (heightWidthRatio < GATE_RATIO_ERROR) {
				// Determines if the rectangle found respects the ratio given in the rules.
				cv::Point2f vertices[4];
				foundRectangle.points(vertices);

				float approximateDistanceWithObject = (FOCAL_LENGTH * DOOR_REAL_HEIGHT * currentFrame.size().height)/(height * CAMERA_SENSOR_HEIGHT);
				// Draws text containing the dimensions on each rectangles.
				std::stringstream ss (std::stringstream::in | std::stringstream::out);
				ss << "Width=" << width << "Height=" << height << " Distance=" << approximateDistanceWithObject << " mm";
				putText(currentFrame, ss.str(), cv::Point(vertices[0].x,vertices[0].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, WHITE_BGRX, 1, CV_AA);

				// Goes through each vertex and connects them to write the edges of the rectangle.
				for(int i = 0; i < 4; ++i) {
					std::cout << "[DEBUG] (" << i << ") = (" << vertices[i].x << ";" << vertices[i].y << ")" << std::endl;
					cv::line(currentFrame, vertices[i], vertices[(i + 1) % 4], BLUE_BGRX, 1, CV_AA);
				}

				drawPointsOfContour(currentFrame, contourAfterPolygonApproximation);
			}
		}
	}
}

/**
 * Draws the points defining a contour.
 *
 * @param frame The cv::Mat object on which the points will be drawn.
 * @param contour The vector containing the points to be drawn.
 */
void drawPointsOfContour(cv::Mat& frame, std::vector<cv::Point> contour) {
	// Draw each single point that forms the polygon.
	for (int j = 0 ; j < contour.size() ; j++ ) {
		cv::Point singlePoint = contour.at(j);
		rectangle(frame, singlePoint, singlePoint, GREEN_BGRX, 8, 8);
	}
}

/**
 * Main function that does the preamble before the video's execution.
*/
int main(int argc, char* argv[]) {
	// Making sure that we have the video provided.
	if (argc != 2) {
		std::cout << "[ERROR] The number of parameters is not correct." << std::endl;
	} else {
		std::cout << "[INFO] The input provided is \"" << argv[1] << "\"" << std::endl;
	}
	
	// Opens the video file given in the parametersCV_CAP_PROP_POS_MSEC.
	cv::VideoCapture videoCapture(argv[1]);

	// Verifies that the video was open successfuly.
	if (!videoCapture.isOpened()) {
		std::cout << "[ERROR] Cannot open the video file" << std::endl;
		return (-1);
	}

	// Gets the video's FPS
	double videoFPS = videoCapture.get(CV_CAP_PROP_FPS);
	int frameWidth = videoCapture.get(CV_CAP_PROP_FRAME_WIDTH);
	int frameHeight = videoCapture.get(CV_CAP_PROP_FRAME_HEIGHT);
	videoCapture.set(CV_CAP_PROP_POS_FRAMES, 35);

	std::cout << "[INFO] FPS=" << videoFPS << std::endl;
	std::cout << "[INFO] Frame width=" << frameWidth << std::endl;
	std::cout << "[INFO] Frame height=" << frameHeight << std::endl;

	// Creates the windows to be used for testing.
	cv::namedWindow(ORIGINAL_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::namedWindow(FILTERED_WINDOW, CV_WINDOW_KEEPRATIO);

	// Sets the size of the windows to be used for testing.
	cv::moveWindow(ORIGINAL_WINDOW, 100, 30);
	cv::moveWindow(FILTERED_WINDOW, 550, 30);

	executeVideo(videoCapture);

	cv::destroyAllWindows();
}
