/*
 * Example 3.19: 2 notch filters in series for recovering corrupted speech recording
 * Notches at f1=900Hz and f2=2700Hz
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

#include "notch2_coeffs.h"
float x[N][2];
uint8_t filter_number = 0;

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
    float yn[2] = { 0.0 };
    int16_t i;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        gpio_set(GPIOD, GPIO10);
        right_in_sample = I2S2_EXT_DR;

        x[0][0] = (float) prbs(8000);
        // x[0][0] = (float) right_in_sample;

        // first filter
        for (i = 0; i < N; ++i) {
            yn[0] += h900[i] * x[i][0];
        }

        x[0][1] = yn[0];

        // second filter
        for (i = 0; i < N; ++i) {
            yn[1] += h2700[i] * x[i][1];
        }

        for (i = N-1; i > 0; --i) {
            x[i][0] = x[i - 1][0];
            x[i][1] = x[i - 1][1];
        }

        right_out_sample = (int16_t) yn[filter_number];
        gpio_clear(GPIOD, GPIO10);
    } else {
        // left channel received (MEMS mic on board)
        gpio_set(GPIOD, GPIO10);
        left_in_sample = I2S2_EXT_DR;
        left_out_sample = left_in_sample;
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

    uint32_t last_button_millis;
    uint32_t button_delay = 500; // need 100ms between button presses

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();
    last_button_millis = millis();

    while (1) {
        if (((millis() - button_delay) >= last_button_millis)
                && !gpio_get(PORT_SWITCH, PIN_SWITCH)) {
            last_button_millis = millis();
            filter_number = (filter_number + 1) % 2;
        }

        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

