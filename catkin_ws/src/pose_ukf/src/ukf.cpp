#include "ukf.h"
#include "matrix_utils.h"
#include "rotation_vector_utils.h"
#include <math.h>
#include <stdio.h>

//Length of the state vector
const double INITIAL_COVARIANCE = 0.0000001;
const double PROCESS_VARIANCE = 0.00000004;
const double MEASUREMENT_VARIANCE = 0.025;



void prettyPrint(double* augCovar, int dim1, int dim2)
{
	for(int i = 0; i < dim1; i++)
	{
		for(int j = 0; j < dim2; j++)
		{
			printf("%e,", augCovar[i*dim2+j]);
		}
		printf("\n");
	}
	printf("\n");
}

ukf::ukf(int dim)
{
	DIM = dim;

	// Create our initial pose estimate and covariance
	// Our internal representation of pose has to be a rotation vector since
	// that is the only chart on SO(3) which is a vector

	//initialize our state, covariance, and sigmas
	augState = new double[DIM]();
	augCovar = new double[DIM*DIM]();
	sigmas = new double[2*DIM*DIM]();
	gammas = new double[2*DIM*DIM]();
	predMsmt = new double[DIM]();
	measCovar = new double[DIM*DIM]();
	crossCovar = new double[DIM*DIM]();

	//They are hopefully now set to zero

	//Now we need to set our initial covariance
	diagonalMatrix(INITIAL_COVARIANCE, augCovar, DIM);

}

void ukf::generateSigmas()
{
	//Here we generate 2*DIM states distributed on
	//a hypersphere around augPose

	//This writes the square root of covar into the first DIM sigmas
	cholesky(augCovar, sigmas, DIM);

	//prettyPrint(sigmas, DIM, DIM);

	//Now we make a copy of it into the second
	vectorCopy(sigmas, &(sigmas[DIM*DIM]), DIM*DIM);

	//Now we scale the sigmas
	scaleVector(sqrt(DIM), sigmas, DIM*DIM);
	scaleVector(-1.0*sqrt(DIM), sigma(DIM), DIM*DIM);

	for (int i = 0; i < 2*DIM; i++)
	{
		addVectors(sigma(i), augState, DIM);
	}
	//So we end up with the augPose added to all the different columns
	//of the matrix square root of augCovar
}

void propogate(double *rotation, double* state)
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

void ukf::recoverPrediction()
{
	averageVectors(sigmas, augState, 2*DIM, DIM);

	subtractMultipleVectors(sigmas, augState, 2*DIM, DIM);

	averageOuterProduct(sigmas, sigmas, augCovar
			,2*DIM, DIM, DIM);

	addDiagonal(augCovar, PROCESS_VARIANCE, DIM);
}

void ukf::predict(double rotation[3])
{
	generateSigmas();
	//prettyPrint(sigma(0), 2*DIM, DIM);
	for (int i = 0; i < 2*DIM; i++)
	{
		propogate(rotation, sigma(i));
	}
	//prettyPrint(sigma(0), 2*DIM, DIM);
	//prettyPrint(augCovar, DIM, DIM);
	recoverPrediction();
	//prettyPrint(augCovar, DIM, DIM);
}

void h(double *sigma, double *gamma)
{
	double gravity[] = {0, 0, 9.8};
	double inverted[3] = {};
	inverse(sigma, inverted);
	rotateThisByThat(gravity, inverted, gamma);
}

void ukf::correct(double acc[3])
{
	generateSigmas();

	//prettyPrint(sigma(0), 2*DIM, DIM);

	//Here we predict the outcome of the acceleration measurement
	//for every sigma and store in the gammas
	for (int i = 0; i < 2* DIM; i++)
	{
		h(sigma(i), gamma(i));
	}

	//prettyPrint(gamma(0), 2*DIM, DIM);
	recoverCorrection(acc);
}

void ukf::recoverCorrection(double *acc)
{

	averageVectors(gammas, predMsmt, 2*DIM, DIM);

	subtractMultipleVectors(sigmas, augState, 2*DIM, DIM);
	subtractMultipleVectors(gammas, predMsmt, 2*DIM, DIM);

	averageOuterProduct(gammas, gammas, measCovar, 2*DIM, DIM, DIM);
	averageOuterProduct(sigmas, gammas, crossCovar, 2*DIM, DIM, DIM);


	//Include measurement variance
	addDiagonal(measCovar, MEASUREMENT_VARIANCE, DIM);


	//prettyPrint(measCovar, DIM, DIM);
	//prettyPrint(crossCovar, DIM, DIM);


	double *gain = new double[DIM*DIM]();
    solve(crossCovar, measCovar, gain, DIM, DIM);


    subtractVectors(acc, predMsmt, DIM);
    leftMultiplyAdd(gain, acc, augState, DIM, DIM, 1);

    //prettyPrint(gain, DIM, DIM);

    scaleVector(-1.0, crossCovar, DIM*DIM);
    transposedMultiplyAdd(crossCovar, gain, augCovar, DIM, DIM, DIM);
    //prettyPrint(augCovar, DIM, DIM);

    delete gain;
}

void fixState(double* state)
{
	double pi = 3.141592653589793238462643383279502884197169399375105820974944;
	double angle = norm(state);
	if (angle > pi)
	{
		//This represents the same rotation, but with norm < pi
		scaleVector(-(2*pi-angle)/angle, state, 3);
	}
}


void ukf::update(double* acc, double* rotation, double *quaternion)
{
	//printf("%3f %3f\n", rotation[1], augState[1]);
	fixState(augState);

    predict(rotation);
    correct(acc);

    quaternionFromRotationVector(quaternion, augState);
}

double *ukf::sigma(int index)
{//Returns a pointer to the desired sigma array
	return vectorIndex(sigmas, index, DIM);
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
