#! /usr/bin/env python

import scipy.signal as signal
import matplotlib.pyplot as plt
import numpy as np

""" Generate an IIR, SOS filter and print it in a header file format"""

PLOT = False
FILTER = "cheby2_bp"

fs = 8000

if FILTER == "elliptic":
    # - 4th-order elliptic lowpass
    # - 2 second-order sections
    # - 1dB passband ripple
    # - 50dB stopband attenuation
    # - fc = 800Hz
    # - fs = 8kHz
    N = 4
    Wn = 800
    sos = signal.iirfilter(4, Wn=Wn, fs=fs, rp=1, rs=50, btype="lowpass", ftype="ellip", output="sos")
elif FILTER == "cheby2_bp":
    # - 18th order Chebyshev Type II bandpass
    # - Fstop1 = 1600Hz
    # - Fstop2 = 2400Hz
    # - 80dB stopband attenuation
    N = 18
    W1 = 1600
    W2 = 2400
    sos = signal.iirfilter(N=N, Wn=[W1, W2], fs=fs, rs=80, btype="bandpass", ftype="cheby2", output="sos")

w_sos, h_sos = signal.sosfreqz(sos, fs=fs)

NUM_SECTIONS = N // 2
header = (
        f"#define NUM_SECTIONS {NUM_SECTIONS}\n"
        f"\n"
        f"float b[NUM_SECTIONS][3] = {{\n"
        )

for s in range(NUM_SECTIONS):
    header += f"\t{{{sos[s][0]}, {sos[s][1]}, {sos[s][2]}}}"
    if s < NUM_SECTIONS - 1:
        header += ",\n"
header += (
        f"\n"
        f"}};\n"
        f"\n"
        f"float a[NUM_SECTIONS][3] = {{\n"
        )
for s in range(NUM_SECTIONS):
    header += f"\t{{{sos[s][3]}, {sos[s][4]}, {sos[s][5]}}}"
    if s < NUM_SECTIONS - 1:
        header += ",\n"
header += f"\n}};\n"

print(header)

if PLOT:
    plt.title("Frequency Response")
    plt.plot(w_sos, 20*np.log10(np.abs(h_sos)), label="second-order sections")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.grid()
    plt.legend()
    plt.show()
