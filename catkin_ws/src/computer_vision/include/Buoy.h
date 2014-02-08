#ifndef CV_BUOY_H
#define CV_BUOY_H

#include "VisibleObject.h"
//#include <sstream>
//#include <random>

// Defines the different constants used:
/** 
 * Camera focal length
 */
static const float BUOY_FOCAL_LENGTH = 8;
/** 
 * Camera sensor height
 */
static const float BUOY_SENSOR_HEIGHT = 6.26f;
/** 
 * Buoy radius, in millimeters
 */
static const float BUOY_RADIUS = 114.3f;
/** 
 * The color of a white sample in the footage for color correction
 */
static const cv::Scalar colorSample(170, 160, 170);
/** 
 * The desired white for the sample after color correction
 */
static const cv::Scalar targetColor(20, 40, 220);

/** 
 * The color correction to apply to get correct whites
 */
static cv::Scalar colorCorrection;

/** 
 * A struct for the buoy color range (in HSV)
 */
struct BuoyColorProfile {
	cv::Scalar lowColor;
	cv::Scalar highColor;

	BuoyColorProfile(cv::Scalar lowColor, cv::Scalar highColor)
		: lowColor(lowColor), highColor(highColor) {}
};

/** 
 * The default red buoy color profiles
 */
static const BuoyColorProfile RED_BUOY(cv::Scalar(-15, 30, 70), cv::Scalar(15, 255, 255));
/** 
 * The default yellow buoy color profiles
 */
static const BuoyColorProfile YELLOW_BUOY(cv::Scalar(10, 30, 40), cv::Scalar(55, 255, 255));

/** 
 * A struct for the information regarding detected buoys (center position and radius)
 */
struct DetectedBuoy {
	cv::Point center;
	int radius;
	float distance;

	DetectedBuoy(cv::Point center, int radius)
		: center(center), radius(radius) {}
};

/** 
 * Buoy class
 */
class Buoy : public VisibleObject {
public:
	Buoy();
	~Buoy();
	std::vector<computer_vision::VisibleObjectData*> retrieveObjectData(cv::Mat& currentFrame);
private:
	virtual void applyFilter(cv::Mat& currentFrame);	
	std::vector<DetectedBuoy> detectBuoys(cv::Mat* image, BuoyColorProfile buoy);
	float getDistance(float radius, int imageHeight);
	bool checkDisk(cv::Mat* image, cv::Point* center, int radius, int sampleCount, float minDensity);
	int getSampleCount(float radius, float coverage);
	void getThresholdImage(cv::Mat* image, BuoyColorProfile buoy, cv::Mat* destination);
	void inRangeWrapped(cv::Mat* source, cv::Scalar low, cv::Scalar high, cv::Mat* destination);
	void applyColorCorrection(cv::Mat* image);
	void colorCorrect(uchar* b, uchar* g, uchar* r);
	int clamp(int n, int low, int high);
	void initColorCorrection();
};

#endif

