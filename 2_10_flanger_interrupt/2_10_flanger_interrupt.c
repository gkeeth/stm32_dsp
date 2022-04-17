/*
 * Listing 2.10: Interrupt-based flanger
 * in ----------------------------(+)--out
 *       |                         |
 *       ----VarDelay >---alpha >---
 *
 * Output is input plus delayed, attenuated version of input. Delay varies over
 * time.
 *
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
#include <libopencm3/cm3/nvic.h>

#include <arm_math.h>

#include "clock.h"
#include "gpio.h"
#include "i2c.h"
#include "i2s.h"
#include "wm8960.h"

#include "pin_definitions.h"


#define TS 0.0000208333333f     // sampling rate 48kHz
#define PERIOD 10               // 0.1Hz delay modulation rate
#define MEAN_DELAY 0.001f       // mean delay in seconds
#define MODULATION_MAG 0.0008f  // delay modulation magnitude (200 ~ 1800us)
#define BUFFER_SIZE 4096
#define ALPHA 0.9f


uint16_t in_ptr = 0;  // sample storage pointer
uint16_t out_ptr = 0; // sample retrieval pointer
float32_t buffer[BUFFER_SIZE];
float32_t t = 0.0f;
float32_t theta;
float32_t xn, yn;
float32_t delay_in_seconds;
uint16_t delay_in_samples;

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_48KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_16, I2S_INTERRUPT);
}

void spi2_isr(void) {
    int16_t right_in_sample = 0;
    int16_t right_out_sample = 0;
    int16_t left_in_sample = 0;
    int16_t left_out_sample = 0;
    int16_t delayed = 0;

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (line in)
        right_in_sample = (int16_t) I2S2_EXT_DR;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);

        right_out_sample = right_in_sample;

        while (!(SPI2_SR & SPI_SR_TXE));
        SPI2_DR = right_out_sample;

    } else {
        // left channel received (mic)
        left_in_sample = (int16_t) I2S2_EXT_DR;
        xn = (float32_t) left_in_sample;
        buffer[in_ptr] = xn;
        in_ptr = (in_ptr + 1) % BUFFER_SIZE;

        t += TS;
        delay_in_seconds = MEAN_DELAY + MODULATION_MAG * arm_sin_f32(2*PI/PERIOD*t);
        delay_in_samples = (uint16_t) (delay_in_seconds * 48000.0f);

        out_ptr = (in_ptr + BUFFER_SIZE - delay_in_samples) % BUFFER_SIZE;
        yn = xn + buffer[out_ptr] * ALPHA; // change to subtraction for HPF -> no bass response
        left_out_sample = (int16_t) yn;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        // for (volatile uint8_t n = 10; n; --n);

        while (!(SPI2_SR & SPI_SR_TXE));
        SPI2_DR = left_out_sample;
    }
    gpio_clear(GPIOD, GPIO11);
}

int main(void) {
    uint32_t last_flash_millis;
    uint32_t blink_delay = 99; // blink every 100ms

    setup();
    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();
    arm_sin_f32(180);

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

