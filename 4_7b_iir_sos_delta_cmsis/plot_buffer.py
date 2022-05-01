#!/usr/bin/env python

import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

PLOT = True

BUFFER_SIZE = 256
N = BUFFER_SIZE
FS = 8000
T = 1.0 / FS

DELTA_AMPLITUDE = 10.0

imp_response = np.fromfile("impulse_response.bin", dtype=np.float32, count=-1, sep="")[::-1]

t = np.linspace(0, N*T, N, endpoint=False)
freq_response = 20 * np.log10(np.abs(fft(imp_response / DELTA_AMPLITUDE)[:N//2])) # take positive frequencies
bins = fftfreq(N, T)[:N//2]

if PLOT:
    plt.figure()
    plt.plot(t, imp_response)
    plt.title("Impulse Response")
    plt.xlabel("time (ms)")
    plt.ylabel("magnitude (dB)")

    plt.figure()
    plt.plot(bins, freq_response)
    plt.title("Frequency Response")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude")

    plt.show()
