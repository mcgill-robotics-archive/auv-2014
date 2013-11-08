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

// Defines the basic colors used in the BGRX color space.
const cv::Scalar GREEN_BGRX = cv::Scalar(0, 255, 0);
const cv::Scalar BLUE_BGRX = cv::Scalar(255, 0, 0);
const cv::Scalar RED_BGRX = cv::Scalar(0, 0, 255);
const cv::Scalar WHITE_BGRX = cv::Scalar(255, 255, 255);
const cv::Scalar MAUVE_BGRX = cv::Scalar(212, 115, 212);

// Defines the different windows used.
const std::string ORIGINAL_WINDOW = "ORIGINAL_WINDOW";
const std::string CONTOURED_WINDOW = "CONTOURED_WINDOW";
const std::string FILTERED_WINDOW = "FILTERED_WINDOW";

void executeVideo(cv::VideoCapture cap);
void applyFilter(cv::Mat& hsvCurrentFrame, cv::Mat& currentFrame);
float computeRectangleRationDifference(float width, float height);
float determineHeight(float width, float height);

/**
 * Goes through each frames of the video and loops back to the begining on the last frame.
*/
void executeVideo(cv::VideoCapture cap) {
	while (1) {
		cv::Mat currentFrame;
		cv::Mat hsvCurrentFrame;

		bool readCurrentFrame = cap.read(currentFrame);
		if (!readCurrentFrame) {
			std::cout << "[INFO] Looping back to the first frame." << std::endl;
			cap.set(CV_CAP_PROP_POS_FRAMES, 35);
			readCurrentFrame = cap.read(currentFrame);
		}

		// Converts the taken frame to HSV.
		cv::cvtColor(currentFrame, hsvCurrentFrame, CV_BGR2HSV);
		std::cout << "Type of hsvCurrentFrame= " << hsvCurrentFrame.type() << std::endl;

		applyFilter(hsvCurrentFrame, currentFrame);

		// Manages the frquency at which the frames are being updated. And closes the windows whenever ESC is pressed.
		if (cv::waitKey(150) == 27) {
			std::cout << "[WARNING] The user has pressed ESC. Terminating the process" << std::endl;
			exit(EXIT_SUCCESS);
		}
	}
}

/**
 * Function that will apply filter on the image so we can detect the door.
*/
void applyFilter(cv::Mat& hsvCurrentFrame, cv::Mat& currentFrame) {
	std::cout << "[INFO] Applying filter to the current frame" << std::endl;
	// Creates the Mat object that will contain the filtered image.
	cv::Mat filteredFrame;
	cv::Mat frameOnlyWithContours = cv::Mat(hsvCurrentFrame.rows, hsvCurrentFrame.cols, CV_8UC3);
	frameOnlyWithContours.setTo(WHITE_BGRX);

	// Apply a Gaussian Blur filter to regularise the pixels from the camera image.
	// This is done in a try to reduce the noise generated by the sensor of the camera.
	cv::GaussianBlur(hsvCurrentFrame, hsvCurrentFrame, cv::Size(KERNEL_SIZE, KERNEL_SIZE), 0, 0);

	// Generates a new Mat object that only contains a certain range of HSV values.
	// Don't forget that we are not using BGRX, but the HSV color space.
	cv::inRange(hsvCurrentFrame, cv::Scalar(0, 0, 0), cv::Scalar(20, 255, 220), filteredFrame);

	// Finds the contours in the images.
	cv::Mat inRangeFrame = filteredFrame.clone();
	// So this vector will contain vectors of points that will form shapes in the image.
	std::vector<std::vector<cv::Point> > detectedContours;
	cv::findContours(filteredFrame, detectedContours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

	for (int i = 0 ; i < detectedContours.size() ; i++) {
		// Gets the contour to analyse in this loop iteration.
		std::vector<cv::Point> contourToAnalyse = detectedContours.at(i);
		// Approximates a polygonal curve(s) with the specified precision.
		std::vector<cv::Point> contourAfterPolygonApproximation;
		approxPolyDP(contourToAnalyse, contourAfterPolygonApproximation, 3, true);

		detectedContours.at(i) = contourAfterPolygonApproximation;

		// Gets the number of points that define this contour.
		int numberOfPointsInContour = contourAfterPolygonApproximation.size();

		// Displays only the contours that have a number of points in the specified range.
		if (numberOfPointsInContour >= 4 && numberOfPointsInContour <= 50) {
			std::cout << "[DEBUG] Number of points in this contour= " << numberOfPointsInContour << std::endl;

			// Finds a rotated rectangle that defines the contour of the object.
			cv::RotatedRect foundRectangle = cv::minAreaRect(cv::Mat(contourAfterPolygonApproximation));

			float width = foundRectangle.size.width;
			float height = foundRectangle.size.height;
			float heightWidthRatio = computeRectangleRationDifference(width, height);

			std::cout << "[DEBUG] Width=" << width << " Height=" << height << std::endl;
			std::cout << "[DEBUG] Ratio=" << width/height << std::endl;
			if (heightWidthRatio < GATE_RATIO_ERROR) {
				// Determines if the rectangle found respects the ratio given in the rules.
				cv::Point2f vertices[4];
				foundRectangle.points(vertices);

				float approximateDistanceWithObject = (FOCAL_LENGTH * DOOR_REAL_HEIGHT * currentFrame.size().height)/(determineHeight(width, height) * CAMERA_SENSOR_HEIGHT);

				// Draws text containing the dimensions on each rectangles.
				std::stringstream ss (std::stringstream::in | std::stringstream::out);
				ss << "Width=" << width << "Height=" << height << " Distance=" << approximateDistanceWithObject << " mm";
				putText(currentFrame, ss.str(), cv::Point(vertices[0].x,vertices[0].y), cv::FONT_HERSHEY_COMPLEX_SMALL, 0.8, WHITE_BGRX, 1, CV_AA);

				// Goes through each vertex and connects them to write the edges of the rectangle.
				for(int i = 0; i < 4; ++i) {
					std::cout << "[DEBUG] (" << i << ") = (" << vertices[i].x << ";" << vertices[i].y << ")" << std::endl;
					cv::line(currentFrame, vertices[i], vertices[(i + 1) % 4], BLUE_BGRX, 1, CV_AA);
					cv::line(frameOnlyWithContours, vertices[i], vertices[(i + 1) % 4], BLUE_BGRX, 1, CV_AA);
				}

				// Draw each single point that forms the polygon.
				for (int j = 0 ; j < numberOfPointsInContour ; j++ ) {
					cv::Point singlePoint = detectedContours.at(i).at(j);
					rectangle(currentFrame, singlePoint, singlePoint, GREEN_BGRX, 8, 8);
				}
			}
		}
	}

	// OpenCV cannot display HSV images properly since it interprets the image as RGB.
	cv::imshow(FILTERED_WINDOW, inRangeFrame);
	cv::imshow(CONTOURED_WINDOW, frameOnlyWithContours);
	// Updates the frame on the window.
	cv::imshow(ORIGINAL_WINDOW, currentFrame);
}

void drawPoints() {

}

float determineHeight(float width, float height) {
	if (width < height) {
			return (height);
	} else {
			return (width);
	}
}

float computeRectangleRationDifference(float width, float height) {
	if (width < height) {
		return (std::abs(height/width - GATE_RATIO));
	} else {
		return (std::abs(width/height - GATE_RATIO));
	}
}

/**
 * Main function that does the preamble before the video's execution.
*/
int main(int argc, char* argv[]) {

	int windowWidth = 500;
	int windowHeight = 500;

	// Making sure that we have the video provided.
	if (argc != 2) {
		std::cout << "[ERROR] The number of parameters is not correct." << std::endl;
	} else {
		std::cout << "[INFO] The input provided is \"" << argv[1] << "\"" << std::endl;
	}
	
	// Opens the video file given in the parametersCV_CAP_PROP_POS_MSEC.
	cv::VideoCapture cap(argv[1]);

	// Verifies that the video was open successfuly.
	if (!cap.isOpened()) {
		std::cout << "[ERROR] Cannot open the video file" << std::endl;
		return (-1);
	}

	// Gets the video's FPS
	double videoFPS = cap.get(CV_CAP_PROP_FPS);
	int frameWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH);
	int frameHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT);
	cap.set(CV_CAP_PROP_POS_FRAMES, 35);

	std::cout << "[INFO] FPS=" << videoFPS << std::endl;
	std::cout << "[INFO] Frame width=" << frameWidth << std::endl;
	std::cout << "[INFO] Frame height=" << frameHeight << std::endl;

	// Creates the windows to be used for testing.
	cv::namedWindow(ORIGINAL_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::namedWindow(CONTOURED_WINDOW, CV_WINDOW_KEEPRATIO);
	cv::namedWindow(FILTERED_WINDOW, CV_WINDOW_KEEPRATIO);

	// Sets the size of the windows to be used for testing.
	cv::moveWindow(ORIGINAL_WINDOW, 100, 30);
	cv::moveWindow(CONTOURED_WINDOW, 100, 400);
	cv::moveWindow(FILTERED_WINDOW, 550, 30);

	executeVideo(cap);

	cv::destroyAllWindows();
}
