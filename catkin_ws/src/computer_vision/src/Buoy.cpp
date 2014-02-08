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
computer_vision::VisibleObjectData* Buoy::retrieveObjectData(Mat& currentFrame) {
	bool isVisible;
	double yawAngle = -90.0;
	double pitchAngle = 90.0;
	double xDistance;
	double yDistance;
	double zDistance;

	// Creates a pointer that will point to a computer_vision::VisibleObjectData object.
	computer_vision::VisibleObjectData* visibleObjectData;

	// Apply filters on the cv::Mat object.
	vector<DetectedBuoy> detected = detectBuoys(currentFrame, RED_BUOY);
	xDistance = detected.center.x;
	yDistance = detected.center.y;
	zDistance = detected.center.z;

	if (!detected.empty()) {

		// Return gathered data to caller
		visibleObjectData->object_type = visibleObjectData->BUOY;
		visibleObjectData->pitch_angle = yawAngle;
		visibleObjectData->yaw_angle = pitchAngle;
		visibleObjectData->x_distance = xDistance;
		visibleObjectData->y_distance = yDistance;
		visibleObjectData->z_distance = zDistance;

		return visibleObjectData;
	} else {
		return (computer_vision::VisibleObjectData*)0;
	}
}
