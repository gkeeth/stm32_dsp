#ifndef CLOCK_H
#define CLOCK_H

uint32_t millis(void);

void sys_tick_handler(void);
void clock_setup(void);

#endif // CLOCK_H
