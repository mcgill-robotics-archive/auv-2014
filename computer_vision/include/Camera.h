#ifndef CV_CAMERA_H
#define CV_CAMERA_H

#include <iostream>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/ros.h>

class Camera  {

	private:
    
    cv::VideoCapture* pVideoCapture;
    cv::Mat* pLastFrame;
    
	public:
    
    Camera(const char* captureSource);
    ~Camera();
    cv::Mat* captureFrame();
};

#endif
