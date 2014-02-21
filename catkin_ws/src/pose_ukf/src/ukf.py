import numpy as np
import numpy.matlib
import scipy
import scipy.linalg
import math
import RotationVectorUtils
import QuaternionUtils

class ukf:
    INITIAL_COVARIANCE = 0.01

    #TODO: Figure out process and measurement variance
    PROCESS_VARIANCE = 0.01
    MEASUREMENT_VARIANCE = 0.01
    #TODO: get delta_t from the ros message
    #TODO: Actually make the variance change based on delta_t
    DELTA_T = 0.026  # In seconds


    def __init__(self):
        self.L = 3 #Length of pose representation
        self.aL = 3*self.L #Length of augmented pose representation
        #Some parameters which should be optimized
        #self.alpha = 0.5
        #self.kappa = 0
        #self.beta = 2
        #self.l = self.alpha * self.alpha * (self.L - self.kappa) - self.L

        #Create our initial pose estimate and covariance
        #Our internal representation of pose has to be a rotation vector since
        #that is the only chart on SO(3) which is a vector
        pose = np.matrix([[0],[0],[0]],float)
        #I have called these bias instead of noise since I am thinking its supposed to
        #be the mean noise
        #Really REALLY not sure about the lengths of these
        processBias = np.matrix([[0]]*self.L)
        measurementBias = np.matrix([[0]]*self.L)
        self.augmentedPose = np.vstack((pose, processBias, measurementBias))

        zero = numpy.zeros((3,3))
        covariance = self.INITIAL_COVARIANCE * np.matlib.eye(self.L)
        self.processCovariance = scipy.linalg.block_diag(zero
                                                         ,self.PROCESS_VARIANCE * np.matlib.eye(self.L)
                                                         ,zero)
        self.measurementCovariance = self.MEASUREMENT_VARIANCE * np.lib.eye(self.L)

        self.augmentedCovariance = scipy.linalg.block_diag(covariance, zero, self.measurementCovariance)
        self.augmentedCovariance += self.processCovariance


    def _matrixRoot(self, matrix):
        tolerance = 10**(-10)
        try:
            return np.matrix(np.linalg.cholesky(matrix))
        except np.linalg.LinAlgError as e:
            eValues = np.linalg.eig(matrix)[0]
            minEValue = min(eValues)
            if math.fabs(minEValue) < tolerance:
                return self._matrixRoot(matrix + tolerance*np.matlib.eye(len(matrix)))
            else:
                raise e


    def _generateSigmas(self):
        #We first find the matrix square root
        #of the augmentedCovariance via cholesky decomposition

        sqrtm = self._matrixRoot(self.augmentedCovariance)

        sigmas = [self.augmentedPose.copy()]
        L = len(self.augmentedPose)
        #lm = 1  # I have NO CLUE what this is supposed to be

        for i in range(L):
            sigmas.append(self.augmentedPose + math.sqrt(self.aL) * sqrtm[...,i])

        #You may think this could be put into the first for loop, but then
        #the sigmas wouldn't be in order, would they
        #....Do they need to be in order?
        for i in range(L):
            sigmas.append(self.augmentedPose - math.sqrt(self.aL) * sqrtm[...,i])

        return sigmas


    def _propagateStates(self, states, omega):

        for state in states:
            # omega is in the body frame
            # we need it in the earth frame
            omegaEarth = RotationVectorUtils.rotateThisByThat(omega, state[0:3])
            state[0:3] = RotationVectorUtils.composeRotations(omegaEarth, state[0:3])


    def recoverPrediction(self, sigmas):
        #Generate weights
        #TODO: Try using the wiki weighting scheme except with self.aL instead of self.L
        #w = 1/(2*(self.L + self.l))
        #ws0 = self.l/(self.L + self.l)
        #wc0 = ws0 + 1 + self.beta - self.alpha * self.alpha
        w0 = 0.0
        w = (1-w0)/(2*self.aL)
        self.augmentedPose = w0 * sigmas[0]
        for sigma in sigmas[1:]:
            self.augmentedPose += w*sigma

        self.augmentedCovariance = w0 * (sigmas[0] - self.augmentedPose) * (sigmas[0] - self.augmentedPose).transpose()
        for sigma in sigmas[1:]:
            self.augmentedCovariance += w * (sigma - self.augmentedPose) * (sigma - self.augmentedPose).transpose()
        self.augmentedCovariance += self.processCovariance


    def predict(self, gyro):
        #Start with previous estimate and covariance

        #Generate sigma points
        sigmas = self._generateSigmas()
        #Propagate sigma points
        self._propagateStates(sigmas, gyro*self.DELTA_T)

        #recover estimate and covariance
        self.recoverPrediction(sigmas)


    def accFromPose(self, pose):
        #Here we calculate what the acceleration is for a pose
        gravity = numpy.matrix([[0],[0],[9.8]])

        return RotationVectorUtils.rotateThisByThat(gravity, RotationVectorUtils.inverse(pose[0:3]))


    def recoverCorrection(self, sigmas, gammas, acc):
        #weights
        #w = 1/(2*(self.L + self.l))
        #ws0 = self.l/(self.L + self.l)
        #wc0 = ws0 + 1 + self.beta - self.alpha * self.alpha
        w0 = 0.0
        w = (1-w0)/(2*self.aL)


        #Recover predicted measurement
        predictedMeasurement = w0 * gammas[0]
        for gamma in gammas[1:]:
            predictedMeasurement += w*gamma
        #And measurement prediction covariance
        measurementCovariance = w0 * (gammas[0] - predictedMeasurement) \
                                   * (gammas[0] - predictedMeasurement).transpose()
        for gamma in gammas[1:]:
            measurementCovariance += w * (gamma - predictedMeasurement) \
                                       * (gamma - predictedMeasurement).transpose()
        measurementCovariance += self.measurementCovariance

        #And cross covariance
        crossCovariance = w0 * (sigmas[0] - self.augmentedPose) \
                             * (gammas[0] - predictedMeasurement).transpose()
        for (gamma,sigma) in zip(gammas[1:],sigmas[1:]):
            crossCovariance += w * (sigma - self.augmentedPose) \
                                 * (gamma - predictedMeasurement).transpose()

        # gain = crossCovariance * measurementCovariance.inverse
        #It might be quicker to do it like this
        gain = np.linalg.solve(measurementCovariance.transpose(), crossCovariance.transpose()).transpose()

        self.augmentedPose += gain * (acc - predictedMeasurement)
        self.augmentedCovariance -= gain * measurementCovariance * gain.transpose()



    def correct(self, acc):
        sigmas = self._generateSigmas()
        gammas = [self.accFromPose(sigma) for sigma in sigmas]

        self.recoverCorrection(sigmas, gammas, np.matrix(acc).transpose())



    def update(self, acc, gyro):
        #Update stuff

        self.predict(numpy.matrix(gyro).transpose())
        self.correct(acc)

        return QuaternionUtils.quaternionFromRotationVector(self.augmentedPose[0:3])

if __name__ == '__main__':
    estimator = ukf()
    #estimate = estimator.update([-0.3828125, -0.229687511921, 7.96250009537]
    #    ,[0.246787205338, -0.00300747156143, 0.010275458917])
    estimate = estimator.update([0,0,9.8],[0.1,0,0])
    estimate = estimator.update([0,0,9.8],[3.041592,0,0])
    estimate = estimator.update([0,0,9.8],[0.01,0,0])
    estimate = estimator.update([0,0,9.8],[0.01,0,0])
    estimate = estimator.update([0,0,9.8],[0.01,0,0])
    estimate = estimator.update([0,0,9.8],[0.01,0,0])
    estimate = estimator.update([0,0,9.8],[0.01,0,0])
    pass
