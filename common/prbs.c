#include "prbs.h"

typedef union {
    uint16_t value;
    struct {
        uint8_t bit0 : 1;
        uint8_t bit1 : 1;
        uint8_t bit2 : 1;
        uint8_t bit3 : 1;
        uint8_t bit4 : 1;
        uint8_t bit5 : 1;
        uint8_t bit6 : 1;
        uint8_t bit7 : 1;
        uint8_t bit8 : 1;
        uint8_t bit9 : 1;
        uint8_t bit10 : 1;
        uint8_t bit11 : 1;
        uint8_t bit12 : 1;
        uint8_t bit13 : 1;
        uint8_t bit14 : 1;
        uint8_t bit15 : 1;
    } bits;
} shift_register;

shift_register reg = {0x0001};

int16_t prbs(int16_t noise_level) {
    int8_t fb;
    fb = (reg.bits.bit15 + reg.bits.bit14 + reg.bits.bit3 + reg.bits.bit1) % 2;
    reg.value = reg.value << 1;
    reg.bits.bit0 = fb;

    if (fb == 0) {
        return -noise_level;
    } else {
        return noise_level;
    }
}
