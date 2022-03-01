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
#include "i2s.h"
#include "wm8960.h"

// I2C pins
#define PORT_I2C GPIOB
#define PIN_SDA GPIO7 // PB7 (AF4)
#define PIN_SCL GPIO8 // PB8 (AF4)

// I2S pins
#define PORT_I2S GPIOB
#define PIN_CK GPIO13 // PB13 (AF5) / alt PB10 (AF5)
#define PIN_WS GPIO12 // PB12 (AF5) / alt PB9 (AF5)
#define PIN_SD_DAC GPIO15 // PB15 (AF5) / alt PC3 (AF5)
#define PIN_SD_ADC GPIO14 // PB14 (AF6) / alt PC2 (AF6)

// pins for GPIO toggling
// (no LEDs on waveshare core407v board, but LEDs on breakout board)
#define PORT_LED GPIOD
#define PIN_LED0 GPIO8
#define PIN_LED1 GPIO9

// pins for toggle switch or button
#define PORT_SWITCH GPIOD
#define PIN_SWITCH GPIO10

static void setup(void) {
    clock_setup();

    // setup GPIOs
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED0 | PIN_LED1);
    // gpio_mode_setup(PORT_SWITCH, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_SWITCH);

    // setup I2C
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    gpio_mode_setup(PORT_I2C, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_SDA | PIN_SCL);
    gpio_set_output_options(PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, PIN_SDA | PIN_SCL);
    gpio_set_af(PORT_I2C, GPIO_AF4, PIN_SDA | PIN_SCL);
    i2c_peripheral_disable(I2C1);
    i2c_reset(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_get_i2c_clk_freq(I2C1) / 1e6);
    i2c_peripheral_enable(I2C1);

    // setup I2S
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI2);
    gpio_mode_setup(PORT_I2S, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_CK | PIN_WS | PIN_SD_DAC | PIN_SD_ADC);
    gpio_set_af(PORT_I2S, GPIO_AF5, PIN_CK | PIN_WS | PIN_SD_DAC);
    gpio_set_af(PORT_I2S, GPIO_AF6, PIN_SD_ADC);

    // 1. set I2SMOD bit in SPI_I2SCFGR.
    //    choose I2S standard through I2SSTD[1:0].
    //    choose data length through DATLEN[1:0].
    //    choose number of bits per channel for the frame through CHLEN.
    //    select mode (tx or rx) for slave through I2SCFG[1:0].
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS << SPI_I2SCFGR_I2SSTD_LSB;
    SPI2_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
    // CHLEN set to 0 (for 16 bits per channel per frame)
    SPI2_I2SCFGR |= SPI_I2SCFGR_CHLEN;
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB;

    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS << SPI_I2SCFGR_I2SSTD_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_CHLEN;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB;

    // 2. set interrupt sources and DMA, if applicable, by writing SPI_CR2
    // nothing to do here for polling

    // 2.5. configure codec
    wm8960_init(16);

    // 3. set I2SE bit in SPI_I2SCFGR
    // enable I2S peripherals after enabling the codec
    // errata 2.7.1 says I2S needs to be enabled while LRCLK is high
    // (for i2s mode) or low (for MSB/LSB justified mode)
    // https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf
    while (!(gpio_port_read(PORT_I2S) & PIN_WS));
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SE;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SE;
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

