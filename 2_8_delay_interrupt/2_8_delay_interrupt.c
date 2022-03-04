/*
 * Listing 2.8: Interrupt-based delay line
 * Output is input plus delayed version of input
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

#define BUFFER_SIZE 2000
volatile int16_t lbuffer[BUFFER_SIZE];
volatile int16_t rbuffer[BUFFER_SIZE];
volatile uint16_t lbuf_index = 0;
volatile uint16_t rbuf_index = 0;

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(DATA_LENGTH_16, CHANNEL_LENGTH_16, I2S_INTERRUPT);
}

void spi2_isr(void) {
    int16_t right_in_sample = 0;
    int16_t right_out_sample = 0;
    int16_t left_in_sample = 0;
    int16_t left_out_sample = 0;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received
        right_in_sample = (int16_t) I2S2_EXT_DR;
        right_out_sample = right_in_sample + rbuffer[rbuf_index];
        rbuffer[rbuf_index] = right_in_sample;
        rbuf_index = (rbuf_index + 1) % BUFFER_SIZE;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);

        while (!(SPI2_SR & SPI_SR_TXE));
        SPI2_DR = right_out_sample;
    } else {
        // left channel received
        left_in_sample = (int16_t) I2S2_EXT_DR;
        left_out_sample = left_in_sample + lbuffer[lbuf_index];
        lbuffer[lbuf_index] = left_in_sample;
        lbuf_index = (lbuf_index + 1) % BUFFER_SIZE;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);

        while (!(SPI2_SR & SPI_SR_TXE));
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

