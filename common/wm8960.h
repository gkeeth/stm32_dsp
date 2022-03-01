#ifndef WM8960_H
#define WM8960_H

// datasheet gives a hex value (0x34 = 0b0110100) and a binary value (0b0011010 = 0x1a)
// Correct address to use is 0x1a; will get left shifted in lib i2c functions.
// hex value is already left shifted with write bit; binary is not shifted.
#define CODEC_ADDRESS 0x1a

// TODO: make static?
void wm8960_write_reg(uint8_t reg_number);

void wm8960_init(uint8_t wordlength);

#endif // WM8960_H
