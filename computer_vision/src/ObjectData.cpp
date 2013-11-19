#include "ObjectData.h"

/**
 * Creates an ObjectData object. The ObjectData class is used to store
 * information on a VisibleObject. After it is created, it is passed to
 * the NavigationPlanner class for further analysis.
 * @param objectType The type of object with which the data is associated.
 * This can be DOOR, BOEY or GROUND_TARGET.
 * @param distance
 * @param verticalAngle
 * @param horizontalAngle
 */
ObjectData::ObjectData(ObjectType objectType, double distance, double verticalAngle, double horizontalAngle) {
	this->objectType = objectType;
	this->distance = distance;
	this->verticalAngle = verticalAngle;
	this->horizontalAngle = horizontalAngle;
}
