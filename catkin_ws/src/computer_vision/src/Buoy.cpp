/**
 * This object is used to detect the buoy object.
 *
 * @author Aleksi Sapon
 */

#include "Buoy.h"

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
vector<computer_vision::VisibleObjectData*> Buoy::retrieveObjectData(Mat& currentFrame) {
	bool isVisible;
	double yawAngle = -90.0;
	double pitchAngle = 90.0;
	double xDistance = -1.0;
	double yDistance = -1.0;
	double zDistance = -1.0;

	// Creates a pointer that will point to a computer_vision::VisibleObjectData object.
	vector<computer_vision::VisibleObjectData*> detectedObjects;
	computer_vision::VisibleObjectData* visibleObjectData;

	// Apply filters on the cv::Mat object.
	vector<DetectedBuoy> detected = detectBuoys(&currentFrame, RED_BUOY);
	
	if (!detected.empty()) {
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
		return detectedObjects;
	} else {
		detectedObjects.push_back((computer_vision::VisibleObjectData*)0);
		return detectedObjects;
	}
}
