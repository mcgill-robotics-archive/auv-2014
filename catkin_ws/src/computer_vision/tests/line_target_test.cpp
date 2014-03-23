#include <gtest/gtest.h>
#include "../include/LineTarget.h"
#include <ros/package.h>

bool compareImage(cv::Mat& correctImage, cv::Mat& testImage) {
	bool valid = true;
	// Iterate through each image, comparing each pixels.
	for (int i = 0; i < correctImage.rows; i++) {
		for (int j = 0; j < correctImage.cols; j++) {
			//std::cout << correctImage.at<int>(i, j) << " " << testImage.at<int>(i, j) << std::endl;
			if (correctImage.at<int>(i, j) != testImage.at<int>(i, j)) valid = false;
		}
	}
	//valid = false;
	return valid;
}

// Simple test ti demonstrate basic unit testing princinples.
TEST(LineTarget, adding) {
	LineTarget* target = new LineTarget();
	EXPECT_EQ(5, target->add(2, 3));
}

// Tests to make sure the filter was properly applied to the given image.
//TEST(LineTarget, applyFilter) {
//	LineTarget* mt = new LineTarget();
//	// Load the raw image in which to apply the filter to.
//	std::string rawPath = ros::package::getPath("computer_vision") + "/tests/line.png";
//	cv::Mat raw = cv::imread(rawPath, CV_LOAD_IMAGE_COLOR);
//	// Apply the filter.
//	mt->applyFilter(raw);
//	// Load the correct version of the filtered image and compare it with the result.
//	std::string filteredPath = ros::package::getPath("computer_vision") + "/tests/filtered_line.png";
//	//cv::Mat correct = cv::imread("~/filtered_line.png");
//	cv::Mat correct = cv::imread(filteredPath);
//	EXPECT_TRUE(compareImage(correct, raw));
//}


//TEST(LineTarget, isVisible) {
//	LineTarget* target = new LineTarget();
//	std::string rawPath = ros::package::getPath("computer_vision") + "/tests/line.png";
//	cv::Mat raw = cv::imread(rawPath, CV_LOAD_IMAGE_COLOR);
//	target->applyFilter(raw);
//	EXPECT_TRUE();
//}

TEST(LineTarget, convertFromPixelsToMetres) {
	LineTarget* target = new LineTarget();
	EXPECT_EQ(30, (int)target->convertFromPixelsToMetres(100, 4));

}

int main(int argc, char **argv) {
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
