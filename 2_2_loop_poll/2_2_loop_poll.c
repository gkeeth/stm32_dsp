/*
 * Example 2.2: Loop-based polling
 * SPI/I2S2 for receive
 * I2S2_ext for transmit
 *
 * I2S2_WS: PB12
 * I2S2_SD: PB15 (Rx)
 * I2S2ext_SD: PB14 (Tx)
 * I2S2_CK: PB13
 *
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/i2c.h>

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

    i2s_setup();

    // 2.5. configure codec
    wm8960_init(16);

    i2s_enable();
}

int main(void) {
    uint16_t right_in_sample = 0;
    uint16_t right_out_sample = 0;
    uint16_t left_in_sample = 0;
    uint16_t left_out_sample = 0;

    uint32_t last_flash_millis;
    uint32_t blink_delay = 99; // blink every 100ms

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();

    while (1) {
        while (!(I2S2_EXT_SR & SPI_SR_RXNE));

        if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
            // left channel received
            left_in_sample = I2S2_EXT_DR;
            // left_out_sample = left_in_sample;
            left_out_sample = 0xAA55;
            while (!(SPI2_SR & SPI_SR_TXE));
            SPI2_DR = left_out_sample;
        } else {
            // right channel received
            right_in_sample = I2S2_EXT_DR;
            // right_out_sample = right_in_sample;
            // if (right_out_sample == 0x7FFF) {
            //     right_out_sample = 0x8000;
            // } else {
            //     right_out_sample = 0x7FFF;
            // }
            right_out_sample = 0x55AA;
            while (!(SPI2_SR & SPI_SR_TXE));
            SPI2_DR = right_out_sample;
        }

        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

