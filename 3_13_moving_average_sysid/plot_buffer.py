#!/usr/bin/env python

import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

PLOT = True

BUFFER_SIZE = 256
N = BUFFER_SIZE
FS = 8000
T = 1.0 / FS

coeffs = np.fromfile("adaptive_filter_coeffs.bin", dtype=np.float32, count=-1, sep="")[::-1]

t = np.linspace(0, N*T, N, endpoint=False)
C = 20 * np.log10(np.abs(fft(coeffs)[:N//2])) # take positive frequencies
bins = fftfreq(N, T)[:N//2]

if PLOT:
    plt.figure()
    plt.plot(t, coeffs, label="Adaptive Filter Coefficients / Impulse Response")
    plt.title("Adaptive Filter Coefficients")
    plt.xlabel("time (ms)")
    plt.ylabel("magnitude")

    plt.figure()
    plt.plot(bins, C, label="Adaptive Filter Frequency Response")
    # plt.semilogy(bins, C, label="Adaptive Filter Frequency Response")
    plt.title("Adaptive Filter Frequency Response")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude")

    plt.show()
