/*
 * Example 3.14: FIR filter with coefficients stored in header file.
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

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"
#include "prbs.h"

#include "pin_definitions.h"

// #include "maf5.h" // 5-point moving average
// #include "lp55.h" // 55-point lowpass, fc = 2kHz
// #include "bs2700.h" // 89-point bandstop with fc = 2700Hz
// #include "bs2700_pyfda.h" // 89-point bandstop with fc = 2700Hz, generated with pyFDA to match bs2700.h
#include "bp1750.h" // 81-point bandpass with fc = 1750Hz
// #include "bp55.h"   // 55-point bandpass, fc = 2kHz
// #include "bs55.h"   // 55-point bandstop, fc = 2kHz
// #include "hp55.h"   // 55-point highpass, fc = 2kHz
// #include "pass2b.h" // 55-point bandpass, 2 passbands
// #include "pass3b.h" // 55-point bandpass, 3 passbands
// #include "pass4b.h" // 55-point bandpass, 4 passbands
// #include "comb14.h" // 55-point comb

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

float x[N];

void spi2_isr(void) {
    float yn = 0.0;
    int16_t i;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        gpio_set(GPIOD, GPIO10);
        right_in_sample = I2S2_EXT_DR;

        // x[0] = (float) prbs(8000);
        x[0] = (float) right_in_sample;
        for (i = 0; i < N; ++i) {
            yn += h[i] * x[i];

        }
        for (i = N-1; i > 0; --i) {
            x[i] = x[i - 1];
        }

        right_out_sample = (int16_t) yn;
        gpio_clear(GPIOD, GPIO10);
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

    for (uint32_t i = 0; i < N; ++i) {
        x[i] = 0.0;
    }

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

