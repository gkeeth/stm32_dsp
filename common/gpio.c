#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include "gpio.h"

#include "pin_definitions.h"

void gpio_setup(void) {
    rcc_periph_clock_enable(RCC_GPIOD);
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED0 | PIN_LED1);

    // for toggling on the logic analyzer
    gpio_mode_setup(GPIOD, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO11);
    gpio_mode_setup(GPIOE, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, GPIO0 | GPIO1);

    // for button input
    // gpio_mode_setup(PORT_SWITCH, GPIO_MODE_INPUT, GPIO_PUPD_NONE, PIN_SWITCH)
    gpio_mode_setup(PORT_SWITCH, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_SWITCH);
}
