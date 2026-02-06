#ifndef UART_H
#define UART_H

#include <stdint.h>
#include "param.h"

void uart0_send_params(const params_t *params);
uint8_t uart0_poll_com_n_gen(void);
void uart1_forward_rx_to_uart0(void);

#endif /* UART_H */
