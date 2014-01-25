/*
 * rotation_vector_utils.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius, Alan Yang
 */

#include <math.h>
#include <algorithm>

#include "rotation_vector_utils.h"

void rotateThisByThat(double result[3], double toRotate[3], double rotation[3]) {
	double angle = norm(rotation);
	
	if (angle == 0.0) {
		std::copy(rotation, rotation + 2, result);
		return;
	}
	
	double dotProduct = dot(toRotate, rotation) / angle;
	
	double cosine = cos(angle),
			sine = sin(angle),
			axis0 = rotation[0] / angle,
			axis1 = rotation[1] / angle,
			axis2 = rotation[2] / angle,
			projection0 = axis0 * dotProduct,
			projection1 = axis1 * dotProduct,
			projection2 = axis2 * dotProduct;
			
	result[0] = (1 - cosine) * projection0 + cosine * toRotate[0] 
		+ sine * (axis1 * (toRotate[2] - projection2) - axis2 * (toRotate[1] - projection1));
		
	result[1] = (1 - cosine) * projection1 + cosine * toRotate[1] 
		+ sine * (axis2 * (toRotate[0] - projection0) - axis0 * (toRotate[2] - projection2));
		
	result[2] = (1 - cosine) * projection2 + cosine * toRotate[2] 
		+ sine * (axis0 * (toRotate[1] - projection1) - axis1 * (toRotate[0] - projection0));
}

double norm(double vector[3]) {
	return sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
}

double dot(double v1[3], double v2[3]) {
	return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] *v2[2];
}

void cross(double result[3], double v1[3], double v2[3]) {
	result[0] = v1[1]*v2[2] - v1[2]*v2[1];
	result[1] = v1[2]*v2[0] - v1[0]*v2[2];
	result[2] = v1[0]*v2[1] - v1[1]*v2[0];
}

void inverse(double result[3], double rotationVector[3]) {
	for (int i = 0; i < 2; i++) {
		result[i] = -1.0 * rotationVector[i];
	}
}

void composeRotations(double result[3], double r1[3], double r2[3]) {
	double q1[] = {1, 0, 0, 0};
	double q2[] = {1, 0, 0, 0};
	
	quaternionFromRotationVector(q1, r1);
	quaternionFromRotationVector(q2, r2);
	
	quaternionMultiply(q1, q2);
	
	rotationVectorFromQuaternion(result, q1);
}

void quaternionFromRotationVector(double quaternion[4], double rotation[3]) {
	double angle = norm(rotation);
	
	if (angle == 0) {
		return;
	}
	
	quaternion[0] = cos(angle / 2.0);
	quaternion[1] = sin(angle / 2.0) * rotation[0] / angle;
	quaternion[2] = sin(angle / 2.0) * rotation[1] / angle;
	quaternion[3] = sin(angle / 2.0) * rotation[2] / angle;
}

void quaternionMultiply(double q[4], double p[4]) {
	double product0 = p[0]*q[0] - p[1]*q[1] - p[2]*q[2] - p[3]*q[3],
			product1 = p[0]*q[1] + p[1]*q[0] + p[2]*q[3] - p[3]*q[2],
			product2 = p[0]*q[2] - p[1]*q[3] + p[2]*q[0] + p[3]*q[1],
			product3 = p[0]*q[3] + p[1]*q[2] - p[2]*q[1] + p[3]*q[0];
	
	q[0] = product0;
	q[1] = product1;
	q[2] = product2;
	q[3] = product3;
}

void rotationVectorFromQuaternion(double rotation[3], double quaternion[4]) {
	std::copy(quaternion + 1, quaternion + 3, rotation);
	double sin = norm(rotation);
	
	if (sin == 0.0) {
		std::fill(rotation, rotation + 2, 0.0);
	}
	
	double angle = atan2(sin, quaternion[0]);
	
	for (int i = 0; i < 2; i++) {
		rotation[i] = rotation[i] * angle / sin;
	}
}
