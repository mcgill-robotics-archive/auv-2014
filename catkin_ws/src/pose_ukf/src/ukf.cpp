/*
 * ukf.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius
 */

#include "ukf.h"
#include "matrix_utils.h"
#include "rotation_vector_utils.h"
#include <math.h>

//Length of the state vector
const double INITIAL_COVARIANCE = 0.01;

const double PROCESS_VARIANCE = 0.01;
const double MEASUREMENT_VARIANCE = 0.01;


ukf::ukf(int dim)
{
	DIM = dim;
	AUGDIM = 3*dim;

	// Create our initial pose estimate and covariance
	// Our internal representation of pose has to be a rotation vector since
	// that is the only chart on SO(3) which is a vector

	//initialize our state, covariance, and sigmas
	augState = new double[AUGDIM]();
	augCovar = new double[AUGDIM*AUGDIM]();
	sigmas = new double[2*AUGDIM*AUGDIM]();
	gammas = new double[2*AUGDIM*DIM]();
	predMsmt = new double[DIM]();
	measCovar = new double[DIM*DIM]();
	crossCovar = new double[AUGDIM*DIM]();

	//They are hopefully now set to zero

	//Now we need to set our initial covariance
	diagonalMatrix(INITIAL_COVARIANCE, augCovar, AUGDIM);

}

void ukf::generateSigmas()
{
	//Here we generate 2*AUGDIM states distributed on
	//a hypersphere around augPose

	//This writes the square root of covar into the first AUGDIM sigmas
	cholesky(augCovar, sigmas, AUGDIM);
	//Now we make a copy of it into the second
	matrixCopy(sigmas, &(sigmas[AUGDIM*AUGDIM]), AUGDIM);

	//Now we scale the sigmas
	scaleVector(sqrt(AUGDIM), sigmas, AUGDIM*AUGDIM);
	scaleVector(sqrt(AUGDIM), sigma(AUGDIM), AUGDIM*AUGDIM);

	for (int i = 0; i < 2*AUGDIM; i++)
	{
		addVectors(sigma(i), augState, AUGDIM);
	}
	//So we end up with the augPose added to all the different columns
	//of the matrix square root of augCovar
}

void ukf::propogateSigmas(double *rotation)
{
	double rotationEarth[3] = {};
	double result[3] = {};
	for (int i = 0; i < 2*AUGDIM; i++)
	{
		rotateThisByThat(rotation, sigma(i), rotationEarth);
		composeRotations(rotationEarth, sigma(i), result);
		for (int j = 0; j < 3; j++)
		{
			sigma(i)[j] = result[j];
		}
	}
}

void ukf::recoverPrediction()
{
	averageVectors(augState, sigmas, 2*AUGDIM, AUGDIM);

	subtractMultipleVectors(sigmas, augState, 2*AUGDIM, AUGDIM);

	averageOuterProductOfVectors(augCovar, sigmas, sigmas
			,2*AUGDIM, AUGDIM, AUGDIM);
	//TODO: Add in process covariance
}

void ukf::predict(double rotation[3])
{
	generateSigmas();
	propogateSigmas(rotation);
	recoverPrediction();
}

void ukf::h(double *sigma, double *gamma)
{
	double gravity[] = {0, 0, 9.8};
	double inverted[3] = {};
	inverse(sigma, inverted);
	rotateThisByThat(gravity, inverted, gamma);
}

void ukf::correct(double acc[3])
{
	generateSigmas();
	//Here we predict the outcome of the acceleration measurement
	//for every sigma and store in the gammas
	for (int i = 0; i < 2* AUGDIM; i++)
	{
		h(sigma(i), gamma(i));
	}

	recoverCorrection(acc);
}

void ukf::recoverCorrection(double *acc)
{

	averageVectors(predMsmt, gammas, 2*AUGDIM, DIM);

	subtractMultipleVectors(sigmas, augState, 2*AUGDIM, AUGDIM);
	subtractMultipleVectors(gammas, predMsmt, 2*AUGDIM, DIM);

	averageOuterProductOfVectors(measCovar, gammas, gammas, 2*AUGDIM, DIM, DIM);
	averageOuterProductOfVectors(crossCovar, sigmas, gammas, 2*AUGDIM, AUGDIM, DIM);


	double *gain = new double[AUGDIM*DIM]();
    solve(measCovar, crossCovar, gain, AUGDIM, DIM);

    //augmentedPose += gain * (acc - predictedMeasurement)
    subtractVectors(acc, predMsmt, DIM);
    leftMultiplyAdd(gain, acc, augState, AUGDIM, DIM, 1);

    //augCovar -= crossCovar * gain.transpose()
    scaleVector(-1.0, crossCovar, DIM*DIM);
    transposedMultiplyAdd(crossCovar, gain, augCovar, AUGDIM, DIM, AUGDIM);

    delete gain;
}

void ukf::update(double* acc, double* rotation)
{
    predict(rotation);

    correct(acc);
    //TODO: return quaternionFromRotationVector(self.augmentedPose[0:3])
}

double *ukf::sigma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(sigmas, index, AUGDIM);
}

double *ukf::gamma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(gammas, index, DIM);
}

/*int main()
{
	ukf filter(3);
	double gyro [3] = {};
	filter.update(gyro, gyro);
	return 0;
}*/
