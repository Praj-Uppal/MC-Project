/*
  A1925661 - Prajwal Uppal
  A1924760 - Vatsal J Thakkar
  A1926225 - Joshua Bonython
*/
#ifndef UART_H
#define UART_H
#include <stdint.h>
void UART_Init(void);
uint8_t UART_Rx(void);
void UART_Tx(uint8_t byte);
#endif