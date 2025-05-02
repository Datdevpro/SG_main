#ifndef UART_SEND_H
#define UART_SEND_H
#include "driver/uart.h"
void uart_init(void);
void send_snore_json(int score) ;
void send_continue_pumping_signal();
void send_stop_pumping_signal();

#endif
