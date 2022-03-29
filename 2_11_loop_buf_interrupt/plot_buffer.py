#!/usr/bin/env python

import numpy as np
from scipy.fft import fft, fftfreq
import matplotlib.pyplot as plt

PLOT = True

BUFFER_SIZE = 256
N = BUFFER_SIZE
FS = 48000
T = 1.0 / FS

rbuf = np.fromfile("rbuffer_binary.bin", dtype=np.float32, count=-1, sep="")
lbuf = np.fromfile("lbuffer_binary.bin", dtype=np.float32, count=-1, sep="")

t = np.linspace(0, N*T, N, endpoint=False)
rf = 2.0/N * np.abs(fft(rbuf)[:N//2]) # normalize and take positive frequencies
lf = 2.0/N * np.abs(fft(lbuf)[:N//2])
bins = fftfreq(N, T)[:N//2]

if PLOT:
    plt.figure()
    plt.plot(t, rbuf, label="Right input buffer (line in)")
    plt.plot(t, lbuf, label="Left input buffer (microphone)")
    plt.legend()
    plt.title("Capture buffers")
    plt.xlabel("time (ms)")
    plt.ylabel("magnitude")

    plt.figure()
    plt.semilogy(bins, rf, label="Right input buffer (line in)")
    plt.semilogy(bins, lf, label="Left input buffer (microphone)")
    plt.legend()
    plt.title("Capture buffers spectrum")
    plt.xlabel("frequency (Hz)")
    plt.ylabel("magnitude")

    plt.show()

