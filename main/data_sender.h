#ifndef UART_SEND_H
#define UART_SEND_H
#include <stdbool.h>
void uart_init(void);
void uart_send_json(int id , bool snore_detected, float ahi);
void uart_receive_response();
void send_flag_start_time(bool flag_snore);
#endif
