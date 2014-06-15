#ifndef rotation_vector_utils_h
#define rotation_vector_utils_h

void rotateThisByThat(double result[3], double toRotate[3], double rotation[3]);
double norm(double vector[3]);
double dot(double v1[3], double v2[3]);
void cross(double result[3], double v1[3], double v2[3]);
void inverse(double result[3], double rotationVector[3]);
void composeRotations(double result[3], double r1[3], double r2[3]);
void quaternionFromRotationVector(double quaternion[4], double rotation[3]);
void quaternionMultiply(double q[4], double p[4]);
void rotationVectorFromQuaternion(double rotation[3], double quaternion[4]);

#endif
