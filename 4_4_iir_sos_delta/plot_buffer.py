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

imp_response_unwarped = np.fromfile("impulse_response_unwarped.bin", dtype=np.float32, count=-1, sep="")[::-1]
imp_response_warped = np.fromfile("impulse_response_warped.bin", dtype=np.float32, count=-1, sep="")[::-1]

t = np.linspace(0, N*T, N, endpoint=False)
freq_response_unwarped = 20 * np.log10(np.abs(fft(imp_response_unwarped / DELTA_AMPLITUDE)[:N//2])) # take positive frequencies
freq_response_warped = 20 * np.log10(np.abs(fft(imp_response_warped / DELTA_AMPLITUDE)[:N//2])) # take positive frequencies
bins = fftfreq(N, T)[:N//2]

if PLOT:
    plt.figure()
    plt.plot(t, imp_response_unwarped, label="Impulse Response (Unwarped)")
    plt.plot(t, imp_response_warped, label="Impulse Response (Warped)")
    plt.title("Impulse Response")
    plt.legend()
    plt.xlabel("time (ms)")
    plt.ylabel("magnitude (dB)")

    plt.figure()
    plt.plot(bins, freq_response_unwarped, label="Frequency Response (Unwarped)")
    plt.plot(bins, freq_response_warped, label="Frequency Response (Warped)")
    plt.title("Frequency Response")
    plt.legend()
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude")

    plt.show()
