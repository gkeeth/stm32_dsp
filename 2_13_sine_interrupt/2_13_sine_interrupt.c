/*
 * Listing 2.13: sine generation with sin function, not lookup table
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

#include <arm_math.h> // for arm_sin_f32()
#include <math.h>     // for sinf()

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"

#include "pin_definitions.h"

#define FS 48000
// #define FS 24000
// #define FS 8000

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    if (FS == 48000) {
        wm8960_init(SAMPLING_RATE_48KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_16, I2S_INTERRUPT);
    } else if (FS == 24000) {
        wm8960_init(SAMPLING_RATE_24KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
    } else if (FS == 8000) {
        wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
    }
}

const float f = 1000.0;
const float A = 10000.0;
float theta = 0.0;
float dtheta;

volatile int16_t right_in_sample = 0;
volatile int16_t right_out_sample = 0;
volatile int16_t left_in_sample = 0;
volatile int16_t left_out_sample = 0;

void spi2_isr(void) {
    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        gpio_set(GPIOD, GPIO10);
        // right channel received
        right_in_sample = I2S2_EXT_DR;
        right_out_sample = right_in_sample;
        right_out_sample = 0x55AA;
    } else {
        // left channel received
        left_in_sample = I2S2_EXT_DR;
        (void) left_in_sample;

        dtheta = 2 * PI * f / FS;
        theta += dtheta;
        if (theta > 2*PI) {
            theta -= 2*PI;
        }
        left_out_sample = (int16_t) (A * sinf(theta)); // 1.8us
        // left_out_sample = (int16_t) (A * arm_sin_f32(theta)); // 1.4us
    }

    // in a real application, an interrupt should be fired on TXE.
    // This blocking loop should be replaced with an if (SPI2_SR & SPI_SR_TXE)
    while (!(SPI2_SR & SPI_SR_TXE));
    if (SPI2_SR & SPI_SR_CHSIDE) {
        SPI2_DR = right_out_sample;
        gpio_clear(GPIOD, GPIO10);
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

