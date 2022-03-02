#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include "i2s.h"

#include "pin_definitions.h"

// setup I2S peripheral as I2S slave. Note: does not enable peripheral.
//
// data_length: number of bits in data word (16, 24, or 32)
// channel_length: number of bits per channel per frame (16 or 32)
// i2s_io_method: io method (polling, interrupt, or DMA)
void i2s_setup(data_length_t data_length, channel_length_t channel_length, i2s_io_method_t i2s_io_method) {
    // Configure I2S pins/clocks
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_SPI2);
    gpio_mode_setup(PORT_I2S, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_CK | PIN_WS | PIN_SD_DAC | PIN_SD_ADC);
    gpio_set_af(PORT_I2S, GPIO_AF5, PIN_CK | PIN_WS | PIN_SD_DAC);
    gpio_set_af(PORT_I2S, GPIO_AF6, PIN_SD_ADC);

    // Setup I2S configuration register
    //   - set I2SMOD bit in SPI_I2SCFGR.
    //   - choose I2S standard through I2SSTD[1:0].
    //   - choose data length through DATLEN[1:0].
    //   - choose number of bits per channel for the frame through CHLEN.
    //   - select mode (tx or rx) for slave through I2SCFG[1:0].
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SMOD;
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS << SPI_I2SCFGR_I2SSTD_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SSTD_I2S_PHILIPS << SPI_I2SCFGR_I2SSTD_LSB;

    if (data_length == DATA_LENGTH_16) {
        SPI2_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
        I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
    } else if (data_length == DATA_LENGTH_24) {
        SPI2_I2SCFGR |= SPI_I2SCFGR_DATLEN_24BIT << SPI_I2SCFGR_DATLEN_LSB;
        I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_24BIT << SPI_I2SCFGR_DATLEN_LSB;
    } else if (data_length == DATA_LENGTH_32) {
        SPI2_I2SCFGR |= SPI_I2SCFGR_DATLEN_32BIT << SPI_I2SCFGR_DATLEN_LSB;
        I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_32BIT << SPI_I2SCFGR_DATLEN_LSB;
    } else {
        // shouldn't get here; use 16
        SPI2_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
        I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_DATLEN_16BIT << SPI_I2SCFGR_DATLEN_LSB;
    }

    if (channel_length == CHANNEL_LENGTH_32) { // CHLEN = 0 for 16 bits per channel per frame
        SPI2_I2SCFGR |= SPI_I2SCFGR_CHLEN;
        I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_CHLEN;
    }

    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_TRANSMIT << SPI_I2SCFGR_I2SCFG_LSB;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SCFG_SLAVE_RECEIVE << SPI_I2SCFGR_I2SCFG_LSB;

    if (i2s_io_method == I2S_INTERRUPT) {
        nvic_enable_irq(NVIC_SPI2_IRQ);
        spi_enable_rx_buffer_not_empty_interrupt(I2S2_EXT_BASE);
    } else if (i2s_io_method == I2S_DMA) {
        // set DMA in SPI_CR2
    }
    // (nothing to do here for polling)
}

// enable I2S. Must be run after configuring codec as I2S master.
void i2s_enable(void) {
    // enable I2S peripherals after enabling the codec
    // errata 2.7.1 says I2S needs to be enabled while LRCLK is high
    // (for i2s mode) or low (for MSB/LSB justified mode)
    // https://www.st.com/resource/en/errata_sheet/es0182-stm32f405407xx-and-stm32f415417xx-device-limitations-stmicroelectronics.pdf
    while (!(gpio_port_read(PORT_I2S) & PIN_WS));
    SPI2_I2SCFGR |= SPI_I2SCFGR_I2SE;
    I2S2_EXT_I2SCFGR |= SPI_I2SCFGR_I2SE;
}
