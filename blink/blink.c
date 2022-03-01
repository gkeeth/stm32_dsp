/*
 * example based on libopencm3 miniblink/systick_blink/button examples
 *
 * original authors:
 * Copyright (C) 2009 Uwe Hermann <uwe@hermann-uwe.de>
 * Copyright (C) 2010 Piotr Esden-Tempski <piotr@esden.net>
 * Copyright (C) 2011 Stephen Caudle <scaudle@doceme.com>
 * Copyright (C) 2012 Karl Palsson <karlp@tweak.net.au>
 * Copyright (C) 2013 Chuck McManis <cmcmanis@mcmanis.com>
 * Copyright (C) 2013 Onno Kortmann <onno@gmx.net>
 * Copyright (C) 2013 Frantisek Burian <BuFran@seznam.cz> (merge)
 *
 * This library is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

// toggle PA0 and PA1
#define PORT_LED GPIOE
#define PIN_LED0 GPIO10
#define PIN_LED1 GPIO11

// switch is on PA2
#define PORT_SWITCH GPIOE
#define PIN_SWITCH GPIO12

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
    rcc_periph_clock_enable(RCC_GPIOE);
    gpio_mode_setup(PORT_LED, GPIO_MODE_OUTPUT, GPIO_PUPD_NONE, PIN_LED0 | PIN_LED1);
    gpio_mode_setup(PORT_SWITCH, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, PIN_SWITCH);

#ifdef USE_MCO
    // put sysclk on MCO2 (PC9). With 4x prescaler we expect 168MHz / 4 = 42MHz
    rcc_periph_clock_enable(RCC_GPIOC);
    gpio_mode_setup(GPIOC, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO9);
    gpio_set_output_options(GPIOC, GPIO_OTYPE_PP, GPIO_OSPEED_100MHZ, GPIO9);
    RCC_CFGR &= ~(RCC_CFGR_MCO2PRE_MASK << RCC_CFGR_MCO2PRE_SHIFT);
    RCC_CFGR |= (RCC_CFGR_MCOPRE_DIV_4 << RCC_CFGR_MCO2PRE_SHIFT); // prescaler = 4
    RCC_CFGR &= ~(RCC_CFGR_MCO2_MASK << RCC_CFGR_MCO2_SHIFT);
    // options for MCO2 source are HSE, PLL, PLLI2S, SYSCLK
    RCC_CFGR |= (RCC_CFGR_MCO2_SYSCLK << RCC_CFGR_MCO2_SHIFT); // source = sysclk
#endif
}

int main(void) {
    uint32_t last_flash_millis;
    uint32_t blink_delay = 2; // 0 -> blink every ms. 1 -> blink every 2ms

    setup();
    gpio_set(PORT_LED, PIN_LED1);
    last_flash_millis = millis();

    while(1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            last_flash_millis = millis();
        }
    }
}
