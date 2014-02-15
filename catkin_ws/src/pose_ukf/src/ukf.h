/*
 * ukf.h
 *
 *  Created on: Jan 22, 2014
 *      Author: mkrogius
 */

#ifndef UKF_H_
#define UKF_H_


class ukf
{
	public:
    	ukf(int dim);
    	void update(double* acc, double* gyro);

	private:
    	int AUGDIM;
    	int DIM;
    	double* augState;
    	double* augCovar;
    	double* predMsmt;
    	double* measCovar;
    	double* crossCovar;
    	double* sigmas;
    	double* gammas;
    	void predict(double rotation[3]);
    	void correct(double acc[3]);
    	void generateSigmas();
    	void propogateSigmas(double *rotation);
    	void recoverPrediction();
    	void recoverCorrection(double acc[3]);
    	void h(double *sigma, double *gamma);
    	double *sigma(int index);
    	double *gamma(int index);


};


#endif /* UKF_H_ */
