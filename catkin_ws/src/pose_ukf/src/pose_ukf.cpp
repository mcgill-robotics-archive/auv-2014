/* Uses the ukf algorithm to do
 * pose estimation.
 *
 * Author: Max Krogius
 */

#include "pose_ukf.h"

const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;

pose_ukf::pose_ukf():filter(3,INITIAL_COVARIANCE
					,PROCESS_VARIANCE,MEASUREMENT_VARIANCE)
{
	const double[] INITIAL_COVARIANCE_MAT = {INITIAL_COVARIANCE,0,0
											,0,INITIAL_COVARIANCE,0
											,0,0,INITIAL_COVARIANCE};
	const double[] PROCESS_VARIANCE_MAT = {PROCESS_VARIANCE,0,0
										  ,0,PROCESS_VARIANCE,0
										  ,0,0,PROCESS_VARIANCE};

	const double[] MEASUREMENT_VARIANCE_MAT = {MEASUREMENT_VARIANCE,0,0
											  ,0,MEASUREMENT_VARIANCE,0
											  ,0,0,MEASUREMENT_VARIANCE};

	filter.setCovariance(INITIAL_COVARIANCE_MAT);
	filter.setProcessVariance();
	filter.setMeasurementVariance();
}

void pose_ukf::propogate(double *rotation, double* state)
{
	//double rotationEarth[3] = {-rotation[0], -rotation[1], -rotation[2]};
	double result[3] = {};
	double tau = 2*3.141592653589793238462643383279502884197169399375105820974944;

	composeRotations(rotation, state, result);

	double angle = norm(result);
	if (angle != 0)
	{
		scaleVector(1/angle, result, 3);
	}

	//We want to choose the rotation vector closest to sigma
	angle += tau*floor(0.5 + (dot(state, result)-angle)/tau);

	for (int j = 0; j < 3; j++)
	{
		state[j] = angle * result[j];
	}
}

void pose_ukf::observe(double *sigma, double *gamma)
{
	double gravity[] = {0, 0, 9.8};
	double inverted[3] = {};
	inverse(sigma, inverted);
	rotateThisByThat(gravity, inverted, gamma);
}
