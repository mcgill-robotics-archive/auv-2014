#ifndef CV_OBJECT_DATA_H
#define CV_OBJECT_DATA_H

enum ObjectType { UNDEFINED = -1, DOOR, BOEY, GROUND_TARGET_1, GROUND_TARGET_2, GROUND_TARGET_3, GROUND_TARGET_4 };

class ObjectData {

	private:

	ObjectType objectType;
	double distance;
	double verticalAngle;
	double horizontalAngle;

	public:

	ObjectData(ObjectType objectType, double distance, double verticalAngle, double horizontalAngle);
};

#endif
