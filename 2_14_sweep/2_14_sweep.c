/*
 * Example 2.14: swept sine generation lookup table
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

#include "pin_definitions.h"

#include "sine8000_table.h"

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
}

#define FS 8000.0
#define N 8000
#define START_FREQ 500.0
#define STOP_FREQ 3800.0
#define START_INCR (START_FREQ * N / FS)
#define STOP_INCR (STOP_FREQ * N / FS)
#define SWEEPTIME 4
#define DELTA_INCR ((STOP_INCR - START_INCR) / (N * SWEEPTIME))

const int16_t amplitude = 10;
float float_index = 0.0;
float float_incr = START_INCR;
int16_t i = 0;

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

        float_incr += DELTA_INCR;
        if (float_incr > STOP_INCR) {
            float_incr = START_INCR;
        }
        float_index += float_incr;
        if (float_index >= N) {
            float_index -= N;
        }
        i = (int16_t) float_index;
        left_out_sample = amplitude * sine8000[i];
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

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

