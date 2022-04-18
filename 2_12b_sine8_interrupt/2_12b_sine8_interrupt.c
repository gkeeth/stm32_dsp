/*
 * Listing 2.12b: lookup-table based sine generation (8kHz sampling rate)
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

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
}

#define LOOPLENGTH 8
int16_t SINETABLE[LOOPLENGTH] = {0, 7071, 10000, 7071, 0 -7071, -10000, -7071};
volatile uint16_t sinen = 0;

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

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);

    } else {
        // left channel received
        left_in_sample = I2S2_EXT_DR;
        left_out_sample = SINETABLE[sinen];
        sinen = (sinen + 1) % LOOPLENGTH;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);
    }
    gpio_clear(GPIOD, GPIO11);

    while (!(SPI2_SR & SPI_SR_TXE));
    if (SPI2_SR & SPI_SR_CHSIDE) {
        SPI2_DR = right_out_sample;
    } else {
        SPI2_DR = left_out_sample;
    }
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

