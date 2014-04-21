#ifndef UKF_H_
#define UKF_H_

void h(double *sigma, double *gamma);
void propogate(double* rotation, double* state);

class ukf
{
	public:
    	ukf(int dim);
    	void update(double* acc, double* gyro, double* quaternion);

	private:
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
    	void recoverPrediction();
    	void recoverCorrection(double acc[3]);

    	double *sigma(int index);
    	double *gamma(int index);


};


#endif /* UKF_H_ */
