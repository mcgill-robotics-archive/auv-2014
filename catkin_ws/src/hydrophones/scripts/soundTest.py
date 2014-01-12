#!/usr/bin/env python

import numpy as np
import pyaudio
import math
import matplotlib.pyplot as plt

FORMAT = pyaudio.paFloat32
SAMPLEFREQ = 96000
BLOCKSIZE = 1024*2*2*2
p = pyaudio.PyAudio()

'''
stream = p.open(format=FORMAT,channels=1,rate=SAMPLEFREQ,input=True,frames_per_buffer=BLOCKSIZE)
data = stream.read(BLOCKSIZE)
timeDomain = np.fromstring(data, 'Float32')
frequencyDomain = np.fft.fft(timeDomain)
amplitude = np.absolute(frequencyDomain) * 2 / BLOCKSIZE
stream.stop_stream()
stream.close()
p.terminate()
plt.plot(amplitude)
plt.show()
'''

a = np.sin(2*10000*math.pi*np.linspace(0, 1, 96000))
frequencyDomain = np.fft.fft(a[:8191])
amplitude = np.absolute(frequencyDomain)
plt.plot(amplitude)
plt.show()