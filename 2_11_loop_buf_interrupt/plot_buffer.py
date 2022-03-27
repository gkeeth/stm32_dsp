#!/usr/bin/env python

import numpy as np
import matplotlib.pyplot as plt

PLOT = True

BUFFER_SIZE = 256
FS = 48000

rbuf = np.fromfile("rbuffer_binary.bin", dtype=np.float32, count=-1, sep="")
lbuf = np.fromfile("lbuffer_binary.bin", dtype=np.float32, count=-1, sep="")

t = np.linspace(0, BUFFER_SIZE / FS, BUFFER_SIZE, endpoint=False)

if PLOT:
    plt.figure()
    plt.plot(t, rbuf, label="Right input buffer (line in)")
    plt.plot(t, lbuf, label="Left input buffer (microphone)")
    plt.legend()
    plt.title("Capture buffers")
    plt.xlabel("time (ms)")
    plt.ylabel("magnitude")

    plt.show()

