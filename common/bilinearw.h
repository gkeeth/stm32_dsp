// bilinearw.h
// second order type 1 Chebyshev LPF with 2dB passband ripple
// and cutoff frequency 1702Hz

#define NUM_SECTIONS 1
float b[NUM_SECTIONS][3]={ {0.15331741793432 ,0.30663483586864,    0.153317417934320} };
float a[NUM_SECTIONS][3]={ {1.0, -0.663876750858361,   0.435937524891090} };
