/*
 * Example 3.13: FIR Moving Average Filter with adaptive filter to identify
 * system response
 * implements y[n] = 1/N * sum(x[n-i]) from i = 0 to N-1
 * SPI/I2S2 for transmit
 * I2S2_ext for receive
 *
 * I2S2_WS: PB12
 * I2S2_SD: PB15 (DAC)
 * I2S2ext_SD: PB14 (ADC)
 * I2S2_CK: PB13
 *
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/cm3/nvic.h>
#include <arm_math.h>

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"
#include "prbs.h"

#include "pin_definitions.h"

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
}

volatile int16_t right_in_sample = 0;
volatile int16_t right_out_sample = 0;
volatile int16_t left_in_sample = 0;
volatile int16_t left_out_sample = 0;

// moving average filter
#define N 5 // number of points in moving average
float h[N];
float x[N];

// adaptive filter
#define BETA 1E-11
#define NUM_TAPS 256
#define BLOCK_SIZE 1
float fir_state_f32[BLOCK_SIZE + NUM_TAPS - 1];
float fir_coeffs_f32[NUM_TAPS] = {0.0};
arm_lms_instance_f32 S;

void spi2_isr(void) {
    float right_in_f32;
    float yn = 0.0;
    int16_t i;

    float adapt_in;
    float adapt_out;
    float desired;
    float error;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        right_in_sample = I2S2_EXT_DR;
        right_in_f32 = (float) right_in_sample;
        adapt_in = (float) prbs(8000);

        gpio_set(GPIOE, GPIO0);

        // S, pSrc, pRef, pOut, pErr, blockSize
        // pSrc: prbs
        // pRef: prbs after moving average filter, DAC, and ADC
        arm_lms_f32(&S, &adapt_in, &right_in_f32, &adapt_out, &error, BLOCK_SIZE);

        x[0] = adapt_in;
        for (i = 0; i < N; ++i) {
            yn += h[i] * x[i];

        }
        for (i = N-1; i > 0; --i) {
            x[i] = x[i - 1];
        }

        gpio_clear(GPIOE, GPIO0);

        right_out_sample = (int16_t) yn;
    } else {
        // left channel received (MEMS mic on board)
        left_in_sample = I2S2_EXT_DR;
        left_out_sample = left_in_sample;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);
        left_out_sample = 0;
    }


    // in a real application, an interrupt should be fired on TXE.
    // This blocking loop should be replaced with an if (SPI2_SR & SPI_SR_TXE)
    while (!(SPI2_SR & SPI_SR_TXE));

    if (SPI2_SR & SPI_SR_CHSIDE) {
        SPI2_DR = right_out_sample;
    } else {
        SPI2_DR = left_out_sample;
    }
    gpio_clear(GPIOD, GPIO11);
}

int main(void) {
    uint32_t last_flash_millis;
    uint32_t blink_delay = 99; // blink every 100ms

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();

    // initialize moving average filter and filter input
    for (uint32_t i = 0; i < N; ++i) {
        h[i] = 1.0 / N;
        x[i] = 0.0;
    }

    // initialize adaptive filter
    arm_lms_init_f32(&S, NUM_TAPS, fir_coeffs_f32, fir_state_f32, BETA, BLOCK_SIZE);

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

