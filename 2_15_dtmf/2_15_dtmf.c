/*
 * Listing 2.15: DTMF (dual tone multi frequency) with lookup table
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

#include <math.h>

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"

#include "pin_definitions.h"

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
}
#define PI 3.14159265359
#define FS 48000.0
#define N 512
#define STEP_770 ((float) ((770 * N) / FS))
#define STEP_1336 ((float) ((1336 * N) / FS))
#define STEP_697 ((float) ((697 * N) / FS))
#define STEP_852 ((float) ((852 * N) / FS))
#define STEP_941 ((float) ((941 * N) / FS))
#define STEP_1209 ((float) ((1209 * N) / FS))
#define STEP_1477 ((float) ((1633 * N) / FS))
#define STEP_1633 ((float) ((1633 * N) / FS))

int16_t table[N];
float n_low = 0.0;
float n_high = 0.0;

volatile int16_t right_in_sample = 0;
volatile int16_t right_out_sample = 0;
volatile int16_t left_in_sample = 0;
volatile int16_t left_out_sample = 0;

void spi2_isr(void) {
    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received
        right_in_sample = I2S2_EXT_DR;
        right_out_sample = right_in_sample;
        right_out_sample = 0x55AA;
    } else {
        // left channel received
        gpio_set(GPIOD, GPIO10);
        left_in_sample = I2S2_EXT_DR;
        (void) left_in_sample;

        left_out_sample = table[(int16_t) n_low] + table[(int16_t) n_high];
        // n_low += STEP_770;
        // n_high += STEP_1477;
        n_low += STEP_697;
        n_high += STEP_1633;

        if (n_low > (float) N) {
            n_low -= N;
        }
        if (n_high > (float) N) {
            n_high -= N;
        }
        gpio_clear(GPIOD, GPIO10);
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
        table[i] = (int16_t) (1000.0 * sin(2 * PI * i / N));
    }

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

