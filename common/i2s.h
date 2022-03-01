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

void i2s_setup(void);
void i2s_enable(void);

#endif // I2S_H
