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
    	void update(double acc[3], double gyro[3]);

	private:
    	int AUGDIM;
    	int DIM;
    	double* augState;
    	double* augCovar;
    	double* sigmas;
    	void predict(double rotation[3]);
    	void correct(double acc[3]);
    	void generateSigmas();
    	void propogateSigmas(double *rotation);
    	void recoverPrediction();
    	double *sigma(int index);

};


#endif /* UKF_H_ */
