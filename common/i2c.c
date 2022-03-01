#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/i2c.h>

#include "i2c.h"

#include "pin_definitions.h"

void i2c_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_I2C1);
    gpio_mode_setup(PORT_I2C, GPIO_MODE_AF, GPIO_PUPD_NONE, PIN_SDA | PIN_SCL);
    gpio_set_output_options(PORT_I2C, GPIO_OTYPE_OD, GPIO_OSPEED_50MHZ, PIN_SDA | PIN_SCL);
    gpio_set_af(PORT_I2C, GPIO_AF4, PIN_SDA | PIN_SCL);
    i2c_peripheral_disable(I2C1);
    i2c_reset(I2C1);
    i2c_set_speed(I2C1, i2c_speed_sm_100k, rcc_get_i2c_clk_freq(I2C1) / 1e6);
    i2c_peripheral_enable(I2C1);
}
