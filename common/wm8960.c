// Driver for WM8960 configuration via i2c

#include <libopencm3/stm32/i2c.h>
#include "wm8960.h"
#include "i2s.h"

static uint16_t reg[] = { // reset values
    0b010010111, //  0. left input volume
    0b010010111, //  1. right input volume
    0b000000000, //  2. LOUT1 volume
    0b000000000, //  3. ROUT1 volume
    0b000000000, //  4. clocking 1
    0b000001000, //  5. ADC/DAC control 1
    0b000000000, //  6. ADC/DAC control 2
    0b000001010, //  7. audio interface 1
    0b111000000, //  8. clocking 2
    0b000000000, //  9. audio interface 2
    0b011111111, // 10. left DAC volume
    0b011111111, // 11. left DAC volume
    0b000000000, // 12. reserved 1
    0b000000000, // 13. reserved 2
    0b000000000, // 14. reserved 3
    0b000000000, // 15. reset - writing to this register resets all registers
    0b000000000, // 16. 3d control
    0b001111011, // 17. ALC1
    0b100000000, // 18. ALC2
    0b000110010, // 19. ALC3
    0b000000000, // 20. noise gate
    0b011000011, // 21. left ADC volume
    0b011000011, // 22. right ADC volume
    0b111000000, // 23. additional control 1
    0b000000000, // 24. additional control 2
    0b000000000, // 25. power management 1
    0b000000000, // 26. power management 2
    0b000000000, // 27. additional control 3
    0b000000000, // 28. anti-pop 1
    0b000000000, // 29. anti-pop 2
    0b000000000, // 30. reserved 4
    0b000000000, // 31. reserved 5
    0b100000000, // 32. ADCL signal path
    0b100000000, // 33. ADCR signal path
    0b001010000, // 34. left out mix 1
    0b001010000, // 35. reserved 6
    0b001010000, // 36. reserved 7
    0b001010000, // 37. right out mix 2
    0b000000000, // 38. mono out mix 1
    0b000000000, // 39. mono out mix 2
    0b000000000, // 40. LOUT2 volume
    0b000000000, // 41. ROUT2 volume
    0b001000000, // 42. MONOOUT volume
    0b000000000, // 43. input boost mixer 1
    0b000000000, // 44. input boost mixer 2
    0b001010000, // 45. bypass 1
    0b001010000, // 46. bypass 2
    0b000000000, // 47. power management 3
    0b000000010, // 48. additional control 4
    0b000110111, // 49. class D control 1
    0b001001101, // 50. reserved 8
    0b010000000, // 51. class D control 3
    0b000001000, // 52. PLL N
    0b000110001, // 53. PLL K 1
    0b000100110, // 54. PLL K 2
    0b011101001  // 55. PLL K 3
};

// TODO: make static?
void wm8960_write_reg(uint8_t reg_number) {
    uint8_t w[2] = {
        (reg_number << 1) | (uint8_t) ((reg[reg_number] >> 8) & 0x1),
        (uint8_t) (reg[reg_number] & 0xFF)
    };

    // TODO: use a different i2c transfer function that is less susceptible to lockup? timeout?
    i2c_transfer7(I2C1, CODEC_ADDRESS, w, 2, 0, 0);
}

// void wm8960_init(uint8_t data_length, uint8_t channel_length) {
void wm8960_init(data_length_t data_length, channel_length_t channel_length, i2s_io_method_t i2s_io_method) {
    i2s_setup(data_length, channel_length, i2s_io_method);
    // TODO: validate settings via headphones, then uncomment speaker enable below
    // could use ADCLRCLK configuration to output sysclk on ADCLRCLK pin

    // enable VREF and fast startup divider
    reg[25] |= (1 << 8) | (1 << 7) | (1 << 6);

    // enable L/R input PGAs
    reg[25] |= (1 << 5) | (1 << 4); // AINL/AINR = 1
    reg[47] |= (1 << 5) | (1 << 4); // LMIC/RMIC = 1

    // setup single-ended mic input L/R via R32/R33 (defaults are good)

    // connect input PGAs to input boost mixers
    reg[32] |= (1 << 3); // LMIC2B = 1, LMN = 1 by default
    reg[33] |= (1 << 3); // RMIC2B = 1, RMN = 1 by default

    // set input PGA volume (LINVOL/RINVOL) - try defaults

    // unmute inputs
    reg[0] &= ~(1 << 7); // LINMUTE = 0
    reg[1] &= ~(1 << 7); // RINMUTE = 0

    // set mic boost (LMICBOOST/RMICBOOST) - try defaults (+0dB)

    // set PGA volume update bit to update volume / unmute
    reg[1] |= (1 << 8);

    // enable ADCs
    reg[25] |= (1 << 3) | (1 << 2); // ADCL/ADCR = 1

    // set ADC volume / unmute - try defaults

    // enable DACs
    reg[26] |= (1 << 8) | (1 << 7); // DACL/DACR = 1
    // configure dac soft mute -- maybe do this last
    reg[5] &= ~(1 << 3); // DACMU = 0
    // output mixers
    reg[47] |= (1 << 3) | (1 << 2); // LOMIX/ROMIX = 1
    reg[34] = 0b100000000; // LD2LO = 1
    reg[37] = 0b100000000; // RD2RO = 1

    // headphone output volume
    reg[2] = 0b001111001; // 0dB
    reg[3] = 0b101111001; // 0dB, with headphone volume update bit set

    // speaker output volume
    reg[40] = 0b001111001; // 0dB
    reg[41] = 0b101111001; // 0dB, with speaker volume update bit set

    // enable outputs
    reg[26] |= (1 << 6) | (1 << 5); // LOUT1/ROUT1 = 1
    reg[26] |= (1 << 4) | (1 << 3); // SPKL/SPKR = 1
    reg[49] |= (1 << 7) | (1 << 6); // SPK_OP_EN = 11 -> SET UP CLOCKS FIRST

    // Audio/Clock/I2S configuration
    // mclk = 24MHz
    // sample rate = 48kHz
    // word length = configurable 16 or 24 bits
    // sysclk = 12.288MHz
    // bclk = 3.072MHz (64 * sampling rate for 24 bits)
    // bclk = 1.536MHz (32 * sampling rate for 16 bits)
    // I2S/justification: left-justified or I2S
    // Codec is master
    //
    // 24 bits
    // 16 bits
    if (data_length == DATA_LENGTH_16) {
        // reg[7] = 0b001000001; // left-justified, 16 bits, master
        reg[7] = 0b001000010; // I2S format, 16 bits, master
    } else if (data_length == DATA_LENGTH_24) {
        // reg[7] = 0b001001001; // left-justified, 24 bits, master
        reg[7] = 0b001001010; // I2S format, 24 bits, master
    } else {
        // should not get here, but use same as 24 bits because that will give
        // enough BCLKs for up to 32 bit words
        // reg[7] = 0b001001001; // left-justified, 24 bits, master
        reg[7] = 0b001001010; // I2S format, 24 bits, master
    }

    // clocking 1
    // CLKSEL = 0b1 -> use pll for sysclk
    // SYSCLKDIV = 0b10 -> sysclk divided by 2 after PLL
    // ADCDIV = 0b000 -> sysclk/256 = 48kHz (default)
    // DACDIV = 0b000 -> sysclk/256 = 48kHz (default)
    reg[4] |= (1 << 0) | (1 << 2); // CLKSEL = 1, SYSCLKDIV = 2

    // clocking 2
    // DCLKDIV = 0b111 -> sysclk/16 = 768kHz (default)
    // BCLKDIV = 0b0100 -> sysclk/4 = 64fs = 3.072MHz (for 24bit words)
    // BCLKDIV = 0b0111 -> sysclk/8 = 32fs = 1.536MHz (for 16bit words)
    if (data_length == DATA_LENGTH_16 && channel_length == CHANNEL_LENGTH_16) {
        reg[8] |= (1 << 2) | (1 << 1) | (1 << 0); // BCLKDIV = 0b0111 (sysclk/8)
        // reg[8] |= (1 << 2); // BCLKDIV = 0b0100 (sysclk/4) -> 32 bits per frame
    } else {
        // 32 BCLKs per channel per frame; enough for up to 32 bits per word
        reg[8] |= (1 << 2); // BCLKDIV = 0b0100 (sysclk/4)
    }

    // pll
    // PLLEN = 1
    // SDM = 1 (fractional mode)
    // PLLPRESCALE = 1 (divide by 2)
    // PLLN = 0x8 (default)
    // PLLK = 0x3126e8
    reg[26] |= (1 << 0); // PLLEN = 1
    reg[52] = 0b000111000; // SDM = 1, PLLPRESCALE = 1, N=0x8
    reg[53] = 0x31;
    reg[54] = 0x26;
    reg[55] = 0xe8;

    // update physical registers
    wm8960_write_reg(15); // reset codec
    wm8960_write_reg(25); // power management 1: AINL/AINR, ADCL/ADCR
    wm8960_write_reg(26); // power management 2: DACL/DACR, LOUT1/ROUT1, SPKL/SPKR, PLLEN
    wm8960_write_reg(47); // power management 3: LMIC/RMIC, LOMIX/ROMIX
    wm8960_write_reg(4);  // clocking 1: CLKSEL, SYSCLKDIV, ADCDIV, DACDIV
    wm8960_write_reg(8);  // clocking 2: BCLKDIV, DCLKDIV
    wm8960_write_reg(52); // SDM, PLLPRESCALE, PLLN
    wm8960_write_reg(53); // PLLK
    wm8960_write_reg(54); // PLLK
    wm8960_write_reg(55); // PLLK
    wm8960_write_reg(7);  // audio interface: I2S options
    wm8960_write_reg(32); // LMIC2B
    wm8960_write_reg(33); // RMIC2B
    wm8960_write_reg(0);  // LINMUTE
    wm8960_write_reg(1);  // RINMUTE
    reg[1] &= ~(1 << 8);  // clear IPVU bit in internal representation -- doesn't need to be written via i2c
    wm8960_write_reg(34); // LD2LO
    wm8960_write_reg(37); // RD2RO
    wm8960_write_reg(2);  // headphone L volume
    wm8960_write_reg(3);  // headphone R volume
    reg[3] &= ~(1 << 8);  // clear VU bit in internal representation -- doesn't need to be written via i2c
    wm8960_write_reg(40); // speaker L volume
    wm8960_write_reg(41); // speaker R volume
    reg[41] &= ~(1 << 8); // clear VU bit in internal representation -- doesn't need to be written via i2c
    // wm8960_write_reg(49); // SPK_OP_EN -- SET UP CLOCKS FIRST
    wm8960_write_reg(5);  // DACMU - do this last

    i2s_enable();
}

