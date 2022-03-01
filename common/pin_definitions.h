#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

// I2C pins
#define PORT_I2C GPIOB
#define PIN_SDA GPIO7 // PB7 (AF4)
#define PIN_SCL GPIO8 // PB8 (AF4)

// I2S pins
#define PORT_I2S GPIOB
#define PIN_CK GPIO13 // PB13 (AF5) / alt PB10 (AF5)
#define PIN_WS GPIO12 // PB12 (AF5) / alt PB9 (AF5)
#define PIN_SD_DAC GPIO15 // PB15 (AF5) / alt PC3 (AF5)
#define PIN_SD_ADC GPIO14 // PB14 (AF6) / alt PC2 (AF6)

// pins for GPIO toggling
// (no LEDs on waveshare core407v board, but LEDs on breakout board)
#define PORT_LED GPIOD
#define PIN_LED0 GPIO8
#define PIN_LED1 GPIO9

// pins for toggle switch or button
#define PORT_SWITCH GPIOD
#define PIN_SWITCH GPIO10

#endif // PIN_DEFINITIONS_H
