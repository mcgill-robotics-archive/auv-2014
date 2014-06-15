#include "ukf.h"
#include "matrix_utils.h"
#include "rotation_vector_utils.h"
//#include <math.h>



void ukf::init(int dim):augState(new double[2*DIM+7*DIM*DIM]())
{
	DIM = dim;

	//initialize our state, covariance, and sigmas
	augCovar = augState + DIM*sizeof(double);
	sigmas = augCovar + DIM*DIM*sizeof(double);
	gammas = gammas + 2*DIM*DIM*sizeof(double);
	predMsmt = gammas + 2*DIM*DIM*sizeof(double);
	measCovar = predMsmt + DIM*sizeof(double);
	crossCovar = measCovar + DIM*DIM*sizeof(double);

}

ukf::ukf(int dim)
{
	init(dim);
}

ukf::ukf(int dim, double initialVariance
		,double processVariance, double measurementVariance)
{
	init(dim);

	const double[] INITIAL_COVARIANCE_MAT = {initialVariance,0,0
											,0,initialVariance,0
											,0,0,initialVariance};
	const double[] PROCESS_VARIANCE_MAT = {processVariance,0,0
										  ,0,processVariance,0
										  ,0,0,processVariance};

	const double[] MEASUREMENT_VARIANCE_MAT = {measurementVariance,0,0
											  ,0,measurementVariance,0
											  ,0,0,measurementVariance};

	setCovariance(INITIAL_COVARIANCE_MAT);
	setProcessVariance(PROCESS_VARIANCE_MAT);
	setMeasurementVariance(MEASUREMENT_VARIANCE_MAT);
}

void ukf::setCovariance(const double covariance[DIM])
{

}

void ukf::generateSigmas()
{
	//Here we generate 2*DIM states distributed on
	//a hypersphere around augPose

	//This writes the square root of covar into the first $DIM sigmas
	cholesky(augCovar, sigmas, DIM);

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
