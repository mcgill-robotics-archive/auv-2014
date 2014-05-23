# TODO: ONLY INTERPOLATE THE RELEVANT RANGE

### IMPORTS
import numpy as np
from objects import mic

INTERPOLATION = 0.001

def interpolate(x,s,u):
    """ Interpolate x with a sinc function """
    if len(x) != len(s):
        raise Exception, 'x and s must be the same length'

    T = s[1] - s[0]
    sincM = np.tile(u, (len(s), 1)) - np.tile(s[:, np.newaxis], (1, len(u)))
    y = np.dot(x, np.sinc(sincM/T))

    return y

def tdoa(origin,other,sampling_frequency):
    """ Compute Time Difference of Arrival """
    # ORDINARY GCC-PHAT
    gcc = np.multiply(origin.freq,np.conj(other.freq))
    phat = np.absolute(np.fft.ifft(np.divide(gcc,np.absolute(gcc))))

    # INTERPOLATION
    sign = 1
    end = 2*len(origin)
    s = np.arange(0,end)
    u = np.arange(0,end,INTERPOLATION)
    phat_interp = interpolate(phat,s,u)
    index = np.argmax(phat_interp)
    if index > end/2:
        sign = -1

    # TIME DIFFERENCE
    other.diff = sign * u[index] / sampling_frequency

    return
