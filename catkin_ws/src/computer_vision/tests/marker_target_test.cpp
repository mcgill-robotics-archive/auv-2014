#include <gtest/gtest.h>
#include "../include/MarkerTarget.h"
#include "../include/LineTarget.h"
#include <ros/package.h>

bool compareImage(cv::Mat& correctImage, cv::Mat& testImage) {
	bool valid = true;
	// Iterate through each image, comparing each pixels.
	for (int i = 0; i < correctImage.rows; i++) {
		for (int j = 0; j < correctImage.cols; j++) {
			std::cout << correctImage.at<int>(i, j) << " " << testImage.at<int>(i, j) << std::endl;
			if (correctImage.at<int>(i, j) != testImage.at<int>(i, j)) valid = false;
		}
	}
	//valid = false;
	return valid;
}

// Simple test ti demonstrate basic unit testing princinples.
TEST(MarkerTarget, adding) {
	MarkerTarget* mt = new MarkerTarget();
	EXPECT_EQ(5, mt->add(2, 3));
}

// Tests to make sure the filter was properly applied to the given image.
TEST(MarkerTarget, applyFilter) {
	MarkerTarget* mt = new MarkerTarget();
	// Load the raw image in which to apply the filter to.
	std::string rawPath = ros::package::getPath("computer_vision") + "/tests/raw_image.png";
	cv::Mat raw = cv::imread(rawPath, CV_LOAD_IMAGE_COLOR);
	// Apply the filter.
	mt->applyFilter(raw);
	// Load the correct version of the filtered image and compare it with the result.
	std::string filteredPath = ros::package::getPath("computer_vision") + "/tests/filtered_image.png";
	cv::Mat correct = cv::imread("~/filtered_image.png");
	EXPECT_TRUE(compareImage(correct, raw));
}

// Makes sure the image was altered properly to contain the contours of the bins and return the correct vector of points.
TEST(MarkerTarget, findBins) {
	// Load the filtered image in which we search for bins.

	// Find bins.

	// Compare the resulting image with the expected resulting image.

	// Compare the resulting vector of corners with the expected corners.

}

// Make sure we only accept rectangles with the correct side-length ratio.
TEST(MarkerTarget, checkRectangleRatio) {
	MarkerTarget* mt = new MarkerTarget();
	// Create a rectangle with the correct side length ratio.
	cv::RotatedRect correct = cv::RotatedRect(cv::Point2f(100, 100), cv::Size2f(100, 200), 30);
	EXPECT_TRUE(mt->checkRectangleRatio(correct));
	// Create a rectangle with an incorrect ratio.
	cv::RotatedRect incorrect = cv::RotatedRect(cv::Point2f(100, 100), cv::Size2f(100, 100), 30);
	EXPECT_FALSE(mt->checkRectangleRatio(incorrect));
}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
