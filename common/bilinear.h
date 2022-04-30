// second order type 1 Chebyshev LPF, 2dB passband ripple, 1500Hz cutoff

#define NUM_SECTIONS 1

float b[NUM_SECTIONS][3]={ {0.12895869, 0.25791738, 0.12895869} };
float a[NUM_SECTIONS][3]={ {1.0, -0.81226498, 0.46166249} };
