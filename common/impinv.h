// IIR filter coefficients for second-order type 1 Chebyshev LPF with 2dB
// passband ripple and 1500 Hz cutoff.
//
// filter generated via impulse invariance method.
//
// transfer function:
// H(z) = (0.48255z-1) / (1 - 0.71624315z-1 + 0.38791310z-2)

#define NUM_SECTIONS 1

float b[NUM_SECTIONS][3] = { {0.0, 0.48255, 0.0} };
float a[NUM_SECTIONS][3] = { {1.0, -0.71624, 0.387913} };
