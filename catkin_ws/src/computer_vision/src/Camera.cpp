#include "Camera.h"

/**
 * Constructs a new Camera object. The camera object is used to handle any
 * camera that is connected to the system.
 * @param captureSource The absolute path to the video file that is going to
 * be transmitted (leave empty to used default camera)
 */
Camera::Camera(const char* captureSource) {

	// Check if video source is specified
	if (strcmp(captureSource, "") == 0)
    	this->pVideoCapture = new cv::VideoCapture(0); // Use default camera
    else
    	this->pVideoCapture = new cv::VideoCapture(captureSource);
    
	// Check if source is readable
 	if (!(*pVideoCapture).isOpened()) {
		ROS_ERROR("Cannot open camera.");
		return;
	}
	// Set video size
	this->pVideoCapture->set(CV_CAP_PROP_FRAME_WIDTH, 640);
	this->pVideoCapture->set(CV_CAP_PROP_FRAME_HEIGHT, 480);

	// Create a new empty opencv image (required, do not remove)
	this->pLastFrame = new cv::Mat();

    /*
    * Don't worry if you see the following message:
    * 
    *     VIDIOC_QUERYMENU: Invalid argument
    *
    * It is coming from v4l and has nothing to do with ROS nor OpenCV.
    * See http://www.ozbotz.org/opencv-install-troubleshooting/
    */
}

/**
 * Releases the memory used by the Camera object.
 */
Camera::~Camera() {
	pVideoCapture->release();
	delete this->pVideoCapture;
	delete this->pLastFrame;
}

/**
 * Captures a frame from the camera and converts it to
 * an opencv image.
 * @return An opencv image
 */
void Camera::captureFrame() {
	bool readFrame = false;

	// Delete last frame and create a new one
	delete pLastFrame;
	pLastFrame = new cv::Mat();

	// Read next frame
    readFrame = (*pVideoCapture).read(*pLastFrame);

    // Check if frame was read
    if (readFrame == false && pVideoCapture != NULL) {
    	/*
    	cv::Exception captureFailed(0, "Camera failed to capture frame", "Camera::captureFrame()", "Camera.cpp", 44);
    	throw captureFailed;
    	*/
		(*pVideoCapture).set(CV_CAP_PROP_POS_FRAMES, 35);
        (*pVideoCapture).read(*pLastFrame);
    }

    // Set capture time
    captureTime = ros::Time::now();
}

cv::Mat* Camera::getLastFrame() {
	return pLastFrame;
}

ros::Time Camera::getCaptureTime() {
	return captureTime;
}


