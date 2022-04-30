#! /usr/bin/env python

import scipy.signal as signal
import matplotlib.pyplot as plt
import numpy as np

# Filter Parameters:
# - 4th-order elliptic lowpass
# - 2 second-order sections
# - 1dB passband ripple
# - 50dB stopband attenuation
# - fc = 800Hz
# - fs = 8kHz

PLOT = False

N = 4
fs = 8000
Wn = 800
b, a = signal.iirfilter(N, Wn=Wn, fs=fs, rp=1, rs=50, btype="lowpass", ftype="ellip")
w, h = signal.freqz(b, a, fs=fs)

# this one should be identical
sos = signal.iirfilter(4, Wn=Wn, fs=fs, rp=1, rs=50, btype="lowpass", ftype="ellip", output="sos")
w_sos, h_sos = signal.sosfreqz(sos, fs=fs)

header = (
        f"#define NUM_SECTIONS {N//2}\n"
        f"\n"
        f"float b[NUM_SECTIONS][3] = {{\n"
        f"\t{{{sos[0][0]}, {sos[0][1]}, {sos[0][2]}}},\n"
        f"\t{{{sos[1][0]}, {sos[1][1]}, {sos[1][2]}}}\n"
        f"}};\n"
        f"\n"
        f"float a[NUM_SECTIONS][3] = {{\n"
        f"\t{{{sos[0][3]}, {sos[0][4]}, {sos[0][5]}}},\n"
        f"\t{{{sos[1][3]}, {sos[1][4]}, {sos[1][5]}}}\n"
        f"}};\n"
        )
print(header)

if PLOT:
    plt.title("Frequency Response")
    plt.plot(w, 20*np.log10(np.abs(h)), label="single section")
    plt.plot(w_sos, 20*np.log10(np.abs(h_sos)), label="second-order sections")
    plt.xlabel("Frequency (Hz)")
    plt.ylabel("Magnitude (dB)")
    plt.grid()
    plt.legend()
    plt.show()
