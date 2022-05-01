/*
 * Example 4.7: IIR filter with cascaded second-order DF1 sections, using
 * CMSIS arm_biquad_cascade_df1_f32()
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
#include "prbs.h"

#include "pin_definitions.h"

// #include "impinv.h"
// #include "bilinear.h"
// #include "bilinearw.h"
#include "elliptic.h"
// #include "bp2000.h"

float coeffs[5*NUM_SECTIONS] = {0};
float state[4*NUM_SECTIONS] = {0};
arm_biquad_casd_df1_inst_f32 S;

static void setup(void) {
    clock_setup();
    gpio_setup();
    i2c_setup();

    wm8960_init(SAMPLING_RATE_8KHZ, DATA_LENGTH_16, CHANNEL_LENGTH_32, I2S_INTERRUPT);
}

volatile int16_t right_in_sample = 0;
volatile int16_t right_out_sample = 0;
volatile int16_t left_in_sample = 0;
volatile int16_t left_out_sample = 0;

void spi2_isr(void) {
    float xn = 0.0;  // input value
    float yn = 0.0;  // output value

    gpio_set(GPIOD, GPIO11);
    if (I2S2_EXT_SR & SPI_SR_CHSIDE) {
        // right channel received (headset mic / line in)
        gpio_set(GPIOE, GPIO0);
        right_in_sample = I2S2_EXT_DR;
        // xn = (float) right_in_sample;
        xn = (float) prbs(8000);
        arm_biquad_cascade_df1_f32(&S, &xn, &yn, 1);

        right_out_sample = (int16_t) yn;
        gpio_clear(GPIOE, GPIO0);
    } else {
        // left channel received (MEMS mic on board)
        left_in_sample = I2S2_EXT_DR;
        left_out_sample = left_in_sample;

        // delay to wait for the next frame to start transmitting
        // in a real program this would be unnecessary because processing
        // the samples takes time
        for (volatile uint8_t n = 10; n; --n);
        left_out_sample = 0;
    }


    // in a real application, an interrupt should be fired on TXE.
    // This blocking loop should be replaced with an if (SPI2_SR & SPI_SR_TXE)
    while (!(SPI2_SR & SPI_SR_TXE));

    if (SPI2_SR & SPI_SR_CHSIDE) {
        SPI2_DR = right_out_sample;
    } else {
        SPI2_DR = left_out_sample;
    }
    gpio_clear(GPIOD, GPIO11);
}

int main(void) {
    uint32_t last_flash_millis;
    uint32_t blink_delay = 99; // blink every 100ms

    setup();

    uint32_t i = 0;
    uint32_t k = 0;
    for (i = 0; i < NUM_SECTIONS; ++i) {
        coeffs[k++] = b[i][0];
        coeffs[k++] = b[i][1];
        coeffs[k++] = b[i][2];
        coeffs[k++] = -a[i][1];
        coeffs[k++] = -a[i][2];
    }
    arm_biquad_cascade_df1_init_f32(&S, NUM_SECTIONS, coeffs, state);

    gpio_set(PORT_LED, PIN_LED0);
    gpio_set(PORT_LED, PIN_LED1);

    last_flash_millis = millis();

    while (1) {
        if ((millis() - blink_delay) > last_flash_millis) {
            gpio_toggle(PORT_LED, PIN_LED0);
            gpio_toggle(PORT_LED, PIN_LED1);
            last_flash_millis = millis();
        }
    }
}

