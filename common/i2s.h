#ifndef I2S_H
#define I2S_H

#include <libopencm3/stm32/spi.h>

// these are not defined in libopencm3's spi.h
// I2S config register
#define I2S2_EXT_I2SCFGR        SPI_I2SCFGR(I2S2_EXT_BASE)
#define I2S3_EXT_I2SCFGR        SPI_I2SCFGR(I2S3_EXT_BASE)
// data register
#define I2S2_EXT_DR             SPI_DR(I2S2_EXT_BASE)
#define I2S3_EXT_DR             SPI_DR(I2S3_EXT_BASE)
// status register
#define I2S2_EXT_SR             SPI_SR(I2S2_EXT_BASE)
#define I2S3_EXT_SR             SPI_SR(I2S3_EXT_BASE)

typedef enum {DATA_LENGTH_16, DATA_LENGTH_24, DATA_LENGTH_32} data_length_t;
typedef enum {CHANNEL_LENGTH_16, CHANNEL_LENGTH_32} channel_length_t;
typedef enum {I2S_POLL, I2S_INTERRUPT, I2S_DMA} i2s_io_method_t;

#define PING 0
#define PONG 1
// buffer holds two channels, so each channel's buffer is I2S_BUFFER_SIZE/2
#define I2S_BUFFER_SIZE 256

void i2s_setup(data_length_t data_length, channel_length_t channel_length, i2s_io_method_t i2s_io_method);
void i2s_enable(void);

#endif // I2S_H
