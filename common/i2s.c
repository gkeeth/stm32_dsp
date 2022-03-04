#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>
#include "i2s.h"

#include "pin_definitions.h"

uint16_t buffer_ping_in[I2S_BUFFER_SIZE];
uint16_t buffer_pong_in[I2S_BUFFER_SIZE];
uint16_t buffer_ping_out[I2S_BUFFER_SIZE];
uint16_t buffer_pong_out[I2S_BUFFER_SIZE];

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
        rcc_periph_clock_enable(RCC_DMA1);
        // I2S2_EXT_RX: stream 3 channel 3
        // SPI2_TX:     stream 4 channel 0
        dma_stream_reset(DMA1, DMA_STREAM3);
        dma_stream_reset(DMA1, DMA_STREAM4);
        dma_channel_select(DMA1, DMA_STREAM3, DMA_SxCR_CHSEL_3);
        dma_channel_select(DMA1, DMA_STREAM4, DMA_SxCR_CHSEL_0);
        dma_set_priority(DMA1, DMA_STREAM3, DMA_SxCR_PL_HIGH);
        dma_set_priority(DMA1, DMA_STREAM4, DMA_SxCR_PL_HIGH);
        dma_enable_double_buffer_mode(DMA1, DMA_STREAM3);
        dma_enable_double_buffer_mode(DMA1, DMA_STREAM4);
        dma_set_memory_size(DMA1, DMA_STREAM3, DMA_SxCR_MSIZE_16BIT);
        dma_set_memory_size(DMA1, DMA_STREAM4, DMA_SxCR_MSIZE_16BIT);
        dma_set_peripheral_size(DMA1, DMA_STREAM3, DMA_SxCR_PSIZE_16BIT);
        dma_set_peripheral_size(DMA1, DMA_STREAM4, DMA_SxCR_PSIZE_16BIT);
        dma_enable_memory_increment_mode(DMA1, DMA_STREAM3);
        dma_enable_memory_increment_mode(DMA1, DMA_STREAM4);
        dma_set_transfer_mode(DMA1, DMA_STREAM3, DMA_SxCR_DIR_PERIPHERAL_TO_MEM);
        dma_set_transfer_mode(DMA1, DMA_STREAM4, DMA_SxCR_DIR_MEM_TO_PERIPHERAL);
        dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM3);
        dma_enable_transfer_complete_interrupt(DMA1, DMA_STREAM4);

        dma_set_number_of_data(DMA1, DMA_STREAM3, I2S_BUFFER_SIZE);
        dma_set_number_of_data(DMA1, DMA_STREAM4, I2S_BUFFER_SIZE);

        dma_set_peripheral_address(DMA1, DMA_STREAM3, (uint32_t) &I2S2_EXT_DR);
        dma_set_peripheral_address(DMA1, DMA_STREAM4, (uint32_t) &SPI2_DR);

        dma_set_memory_address(DMA1, DMA_STREAM3, (uint32_t) buffer_ping_in);
        dma_set_memory_address(DMA1, DMA_STREAM4, (uint32_t) buffer_ping_out);
        dma_set_memory_address_1(DMA1, DMA_STREAM3, (uint32_t) buffer_pong_in);
        dma_set_memory_address_1(DMA1, DMA_STREAM4, (uint32_t) buffer_pong_out);

        spi_enable_rx_dma(I2S2_EXT_BASE);
        spi_enable_tx_dma(SPI2);

        nvic_enable_irq(NVIC_DMA1_STREAM3_IRQ);
        nvic_enable_irq(NVIC_DMA1_STREAM4_IRQ);

        dma_enable_stream(DMA1, DMA_STREAM3);
        dma_enable_stream(DMA1, DMA_STREAM4);
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
