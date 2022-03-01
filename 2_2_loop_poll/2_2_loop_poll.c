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
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include "i2s.h"
#include "wm8960.h"

// I2C pins
#define PORT_I2C GPIOB
#define PIN_SDA GPIO7 // PB7 (AF4)
#define PIN_SCL GPIO8 // PB8 (AF4)

// #define SSD1306_ADDRESS 0x3c // display address

// I2S pins
#define PORT_I2S GPIOB
#define PIN_CK GPIO13 // PB13 (AF5) / alt PB10 (AF5)
#define PIN_WS GPIO12 // PB12 (AF5) / alt PB9 (AF5)
#define PIN_SD_RX GPIO15 // PB15 (AF5) / alt PC3 (AF5) TODO: swap this with TX?
#define PIN_SD_TX GPIO14 // PB14 (AF6) / alt PC2 (AF6) TODO: swap this with TX?

// pins for GPIO toggling (no LEDs on waveshare core407v board)
#define PORT_LED GPIOD
#define PIN_LED0 GPIO8
#define PIN_LED1 GPIO9

// pins for toggle switch or button
#define PORT_SWITCH GPIOD
#define PIN_SWITCH GPIO10

volatile uint32_t counter = 0;

void sys_tick_handler(void) {
    ++counter;
}

uint32_t millis(void);
uint32_t millis(void) {
    return counter;
}

static void setup(void) {
    // use external 8MHz crystal to generate 168MHz clock
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    STK_CVR = 0; // clear systick to start immediately

    // every 1ms (1000 Hz)
    systick_set_frequency(1000, rcc_ahb_frequency);
    systick_counter_enable();
    systick_interrupt_enable();

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

#if 1
    // setup I2S
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI2); // TODO: rcc_periph_clock_enable(RCC_I2S2_ext); ??
    gpio_mode_setup(PORT_I2S, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_CK | PIN_WS | PIN_SD_RX | PIN_SD_TX);
    gpio_set_af(PORT_I2S, GPIO_AF5, PIN_CK | PIN_WS | PIN_SD_RX);
    gpio_set_af(PORT_I2S, GPIO_AF6, PIN_SD_TX);

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
    // SPI2_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB; // original, wrong
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB; // fixed?

    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS << SPI_I2SCFGR_I2SSTD_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_CHLEN;
    // I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB; // original, wrong
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB; // fixed?

    // 2. set interrupt sources and DMA, if applicable, by writing SPI_CR2
    // nothing to do here for polling

    // 3. set I2SE bit in SPI_I2SCFGR
    // SPI2_I2SCFGR |= SPI_I2SCFGR_I2SE;
    // I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SE;
#endif

    // TODO: configure the codec
}

int main(void) {
    uint16_t right_in_sample = 0;
    uint16_t right_out_sample = 0;
    uint16_t left_in_sample = 0;
    uint16_t left_out_sample = 0;

    uint8_t loop_i2c = 0;

    uint32_t last_flash_millis;
    uint32_t last_i2c_millis;
    uint32_t i2c_delay = 1; // 0 -> i2c every ms. 1 -> i2c every 2ms
    uint32_t blink_delay = 99; // blink every 100ms

    uint8_t dummy [] = {0xde, 0xad, 0xbe, 0xef};

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();
    last_i2c_millis = last_flash_millis;

    wm8960_init(16);
    // enable I2S peripherals after enabling the codec
    // errata 2.7.1 says I2S needs to be enabled while LRCLK is high
    // (for i2s mode) or low (for MSB/LSB justified mode)
    // https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf
    while (!(gpio_port_read(PORT_I2S) & (1 << 12))); // TODO: make parametric?
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SE;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SE;

    while (1) {
        if (loop_i2c && (millis() - i2c_delay + 1) > last_i2c_millis) {
            // wm8960 only accepts 2 bytes at a time (7 address bits, 9 data bits)
            //   1. 7 bit register address + data bit 8
            //   2. data bits 7:0
            i2c_transfer7(I2C1, CODEC_ADDRESS, dummy, 2, 0, 0);
            i2c_transfer7(I2C1, CODEC_ADDRESS, dummy + 2, 2, 0, 0);
            last_i2c_millis = millis();
        }

#define TX_I2S_SR SPI2_SR
#define RX_I2S_SR I2S2_EXT_SR
#define TX_I2S_DR SPI2_DR
#define RX_I2S_DR I2S2_EXT_DR
        while (!(RX_I2S_SR & SPI_SR_RXNE));
        if (RX_I2S_SR & SPI_SR_CHSIDE) {
            // left channel received
            left_in_sample = RX_I2S_DR;
            // left_out_sample = left_in_sample;
            left_out_sample = 0xAA55;
            while (!(TX_I2S_SR & SPI_SR_TXE));
            TX_I2S_DR = left_out_sample;
        } else {
            // right channel received
            right_in_sample = RX_I2S_DR;
            // right_out_sample = right_in_sample;
            // if (right_out_sample == 0x7FFF) {
            //     right_out_sample = 0x8000;
            // } else {
            //     right_out_sample = 0x7FFF;
            // }
            right_out_sample = 0x55AA;
            while (!(TX_I2S_SR & SPI_SR_TXE));
            TX_I2S_DR = right_out_sample;
        }

        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

