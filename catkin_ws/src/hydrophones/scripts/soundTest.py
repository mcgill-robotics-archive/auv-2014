#!/usr/bin/env python

import numpy as np
import pyaudio
import math
import matplotlib.pyplot as plt

FORMAT = pyaudio.paFloat32
SAMPLEFREQ = 96000
BLOCKSIZE = 1024
p = pyaudio.PyAudio()
'''
stream = p.open(format=FORMAT,channels=1,rate=SAMPLEFREQ,input=True,frames_per_buffer=BLOCKSIZE)
data = stream.read(BLOCKSIZE)
timeDomain = np.fromstring(data, 'Float32')
frequencyDomain = np.fft.fft(timeDomain)
amplitude = np.absolute(frequencyDomain) / 1024
stream.stop_stream()
stream.close()
p.terminate()
plt.plot(amplitude)
plt.show()
'''
bits = 15
a = np.sin(2*8192*math.pi*np.linspace(0, 1, 2*96000))
frequencyDomain = np.fft.fft(a[:2**bits-1])
amplitude = np.absolute(frequencyDomain)/2**bits
plt.plot(amplitude)
plt.show()

#f(x) = 0.0633x - 0.004
