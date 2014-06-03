### IMPORTS
import numpy as np
from point import Point

BUFFERSIZE = 1024
RFFT_LENGTH = BUFFERSIZE / 2 + 1

class Mic(object):
    ''' Microphone class '''
    __slots__ = ['time', 'freq', 'magn',
                 'pos', 'diff']

    def __init__(self, pos):
        ''' Initializes mic object '''
        self.reset()
        self.pos = pos

    def reset(self):
        ''' Resets mic object '''
        self.time = np.zeros(BUFFERSIZE,np.float)
        self.freq = np.zeros(RFFT_LENGTH,np.float)
        self.magn = np.zeros(RFFT_LENGTH,np.float)
        self.diff = 0

    def compute_fft(self):
        ''' FFTs time domain '''
        self.freq = np.fft.rfft(self.time)

    def compute_magnitude(self):
        ''' Computes magnitude '''
        self.magn = np.absolute(self.frequency)
