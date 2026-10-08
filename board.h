#ifndef BOARD_H
#define BOARD_H

#include "maxm86161.h"

uint8_t board_clock_init(void);
void board_pins_init(void);
void board_sensor_enable(uint8_t on);
void board_uart_init(void);
uint8_t board_uart_char(char value);

#endif
