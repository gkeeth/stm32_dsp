/*
 * Example 4.4: IIR filter with cascaded second-order DFII sections.
 * Uses impulse sequence as input to estimate frequency response.
 *
 * implements:
 * w[n] = x[n] - a1 * w[n-1] - a2 * w[n-2]
 * y[n] = b0 * w[n] + b1 * w[n-1] + b2 * w[n-2]
 *
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

#define BUFFER_SIZE 256
#define AMPLITUDE 10000.0f
#include "impinv.h"
// #include "elliptic.h"
float w[NUM_SECTIONS][2] = {0};
float dimpulse[BUFFER_SIZE];
float response[BUFFER_SIZE];
int16_t bufptr = 0;

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

void spi2_isr(void) {
    float input = 0.0;  // input value
    float wn = 0.0;     // intermediate value
    float yn = 0.0;     // output value
    int16_t section;    // second order section number

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        gpio_set(GPIOE, GPIO0);
        right_in_sample = I2S2_EXT_DR;
        // input = (float) right_in_sample;
        // input = (float) prbs(8000);
        input = dimpulse[bufptr];

        for (section = 0; section < NUM_SECTIONS; ++section) {
            // w[n] = x[n] - a1 * w[n-1] - a2 * w[n-2]
            // y[n] = b0 * w[n] + b1 * w[n-1] + b2 * w[n-2]
            wn = input - a[section][1] * w[section][0] - a[section][2] * w[section][1];
            yn = b[section][0] * wn + b[section][1] * w[section][0] + b[section][2] * w[section][1];
            w[section][1] = w[section][0];
            w[section][0] = wn;
            input = yn;
        }
        response[bufptr] = yn;
        bufptr = (bufptr + 1) % BUFFER_SIZE;

        right_out_sample = (int16_t) (yn * AMPLITUDE);
        gpio_clear(GPIOE, GPIO0);
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

    dimpulse[0] = 10.0;
    for (int i = 1; i < BUFFER_SIZE; ++i) {
        dimpulse[i] = 0.0;
    }

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

