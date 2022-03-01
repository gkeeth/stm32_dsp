#include <libopencm3/stm32/rcc.h>
#include <libopencm3/cm3/systick.h>

#include "clock.h"

volatile uint32_t counter = 0;

void sys_tick_handler(void) {
    ++counter;
}

uint32_t millis(void) {
    return counter;
}

void clock_setup(void) {
    // use external 8MHz crystal to generate 168MHz clock
    rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
    STK_CVR = 0; // clear systick to start immediately

    // systick fires every 1ms (1000 Hz)
    systick_set_frequency(1000, rcc_ahb_frequency);
    systick_counter_enable();
    systick_interrupt_enable();
}
