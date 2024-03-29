/*
 * Example 3.21: DMA-based FIR
 * SPI/I2S2 for transmit
 * I2S2_ext for receive
 *
 * I2S2_WS: PB12
 * I2S2_SD: PB15 (DAC)
 * I2S2ext_SD: PB14 (ADC)
 * I2S2_CK: PB13
 *
 */

#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/spi.h>
#include <libopencm3/stm32/dma.h>
#include <libopencm3/cm3/nvic.h>

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"
#include "prbs.h"

// #include "bp1750.h"
#include "pass4b.h"

#include "pin_definitions.h"

// in i2s.c
extern int16_t buffer_ping_in[I2S_BUFFER_SIZE];
extern int16_t buffer_pong_in[I2S_BUFFER_SIZE];
extern int16_t buffer_ping_out[I2S_BUFFER_SIZE];
extern int16_t buffer_pong_out[I2S_BUFFER_SIZE];

// track PING or PONG
uint8_t rx_processing_buffer;
uint8_t tx_processing_buffer;

volatile uint8_t rx_buffer_full = 0;
volatile uint8_t tx_buffer_empty = 0;

void process_buffer(void);

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_DMA);
}

void dma1_stream3_isr(void) {
    // I2S2_EXT_RX transfer complete
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM3, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM3, DMA_TCIF);
        if (dma_get_target(DMA1, DMA_STREAM3)) {
            // currently DMAing data into PONG buffer, so process PING
            rx_processing_buffer = PING;
        } else {
            // process PONG
            rx_processing_buffer = PONG;
        }
        rx_buffer_full = 1;
    }
}

void dma1_stream4_isr(void) {
    // SPI2_TX transfer complete
    if (dma_get_interrupt_flag(DMA1, DMA_STREAM4, DMA_TCIF)) {
        dma_clear_interrupt_flags(DMA1, DMA_STREAM4, DMA_TCIF);
        if (dma_get_target(DMA1, DMA_STREAM4)) {
            // currently DMAing data into PONG buffer, so process PING
            tx_processing_buffer = PING;
        } else {
            // process PONG
            tx_processing_buffer = PONG;
        }
        tx_buffer_empty = 1;
    }
}

void process_buffer(void) {
    uint32_t i;         // index into DMA buffer
    uint32_t j;         // index into filter coeffs
    static uint32_t k;  // index into x circular buffer
    int16_t *rxbuf;
    int16_t *txbuf;
    float x[I2S_BUFFER_SIZE/2];
    float y[I2S_BUFFER_SIZE/2];

    if (rx_processing_buffer == PING) {
        rxbuf = buffer_ping_in;
    } else {
        rxbuf = buffer_pong_in;
    }

    if (tx_processing_buffer == PING) {
        txbuf = buffer_ping_out;
    } else {
        txbuf = buffer_pong_out;
    }

    gpio_set(GPIOE, GPIO0);
    for (i = 0; i < I2S_BUFFER_SIZE/2; i++) {
        // left channel
        *txbuf++ = *rxbuf++;

        // right channel
        y[i] = 0.0;
        // x[i] = (float) *rxbuf;
        x[k] = (float) prbs(8000);
        k = (k + 1) % N;

        for (j = 0; j < N; ++j) {
            y[i] += h[j] * x[k];
            k = (k + 1) % N;
        }

        rxbuf++;
        *txbuf++ = (int16_t) y[i];
    }
    gpio_clear(GPIOE, GPIO0);
    rx_buffer_full = 0;
    tx_buffer_empty = 0;
}

int main(void) {
    uint32_t last_flash_millis;
    uint32_t blink_delay = 99; // blink every 100ms

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();

    gpio_clear(GPIOD, GPIO11);
    while (1) {
        if (rx_buffer_full && tx_buffer_empty) {
            gpio_set(GPIOD, GPIO11);
            process_buffer();
            gpio_clear(GPIOD, GPIO11);
        }

        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

