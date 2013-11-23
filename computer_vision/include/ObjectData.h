#ifndef CV_OBJECT_DATA_H
#define CV_OBJECT_DATA_H

enum ObjectType { DOOR, BOEY, GROUND_TARGET };

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
