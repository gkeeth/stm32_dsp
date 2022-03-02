#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "gpio.h"

#include "pin_definitions.h"

void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOD);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED0 | PIN_LED1);
    // gpio_mode_setup(PORT_SWITCH, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_SWITCH);

    // for toggling on the logic analyzer
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
}
