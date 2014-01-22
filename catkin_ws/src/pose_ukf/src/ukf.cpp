/*
 * ukf.cpp
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius
 */

#include "ukf.h"
#include "matrix_utils.h"
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
{//TODO: Implement this
	/*
	double rotationEarth[3] = {};
	for (int i = 0; i < 2*AUGDIM; i++)
	{
		rotateThisByThat(rotation, sigma(i), rotationEarth);
		composeRotations(omegaEarth, sigma(i), sigma(i)));
	}*/
}

void ukf::recoverPrediction()
{
	averageVectors(augState, sigmas, 2*AUGDIM, AUGDIM);
	averageOuterProductOfVectors(augCovar, sigmas, 2*AUGDIM, AUGDIM);
	//TODO: Add in process covariance
}

void ukf::predict(double rotation[3])
{
	generateSigmas();
	propogateSigmas(rotation);
	recoverPrediction();
}

void ukf::correct(double acc[3])
{

}

void ukf::update(double acc[3], double rotation[3])
{
    predict(rotation);
    //correct(acc);
    //return quaternionFromRotationVector(self.augmentedPose[0:3])
}

double *ukf::sigma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(sigmas, index, AUGDIM);
}

int main()
{
	ukf filter(3);
	double gyro [3] = {};
	filter.update(gyro, gyro);
	return 0;
}
