/*
 * Example 3.23: Comparison of 4 different inner loops for applying the filter.
 * Methods are cycled through by pressing the button.
 * Measured times:
 * 1. 21.2us
 * 2. 8.8us
 * 3. 8.5us
 * 4. 8.7us
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

#include "bp1750.h"
uint8_t filter_method = 0;
float x[2*N];

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
    float yn = 0.0;
    int16_t i;
    static uint32_t k = 0;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        right_in_sample = I2S2_EXT_DR;

        switch (filter_method) {
            case 0: // simple convolution sum, x values shifted each sample
                gpio_set(GPIOE, GPIO0 | GPIO1);
                x[0] = (float) prbs(8000);
                for (i = 0; i < N; ++i) {
                    yn += h[i] * x[i];

                }
                for (i = N-1; i > 0; --i) {
                    x[i] = x[i - 1];
                }
                gpio_clear(GPIOE, GPIO0 | GPIO1);
                break;
            case 1: // x in circular buffer, buffer pointer incremented with %
                gpio_set(GPIOE, GPIO0 | GPIO1);
                x[k] = (float) prbs(8000);
                k = (k + 1) % N;
                for (i = 0; i < N; ++i) {
                    yn += h[i] * x[k];
                    k = (k + 1) % N;
                }
                gpio_clear(GPIOE, GPIO0 | GPIO1);
                break;
            case 2: // x in circular buffer, buffer pointer incremented without %
                gpio_set(GPIOE, GPIO0 | GPIO1);
                x[k++] = (float) prbs(8000);
                if (k >= N) {
                    k = 0;
                }
                for (i = 0; i < N; ++i) {
                    yn += h[i] * x[k++];
                    if (k >= N) {
                        k = 0;
                    }
                }
                gpio_clear(GPIOE, GPIO0 | GPIO1);
                break;
            case 3: // x in circular buffer, fewer checks on buffer pointer, but buffer twice as big
                gpio_set(GPIOE, GPIO0 | GPIO1);
                x[k] = (float) prbs(8000);
                x[k+N] = x[k];
                k = (k + 1) % N;
                if (k >= N) {
                    k = 0;
                }
                for (i = 0; i < N; ++i) {
                    yn += h[i] * x[k + i];
                    k = (k + 1) % N;
                }
                gpio_clear(GPIOE, GPIO0 | GPIO1);
                break;
        }

        right_out_sample = (int16_t) yn;
    } else {
        // left channel received (MEMS mic on board)
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
            filter_method = (filter_method + 1) % 4;
        }

        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

