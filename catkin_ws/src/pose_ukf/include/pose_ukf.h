#ifndef POSE_UKF_H_
#define POSE_UKF_H_

#include "ukf.h"


class pose_ukf
{
	public:
	    pose_ukf();
		void update(double* acc, double* gyro, double* quaternion);

	private:
		void observe(double *sigma, double *gamma);
		void propogate(double* rotation, double* state);
		ukf filter;
};

#endif
