
#ifndef  __USER_UART_H
#define  __USER_UART_H

#include "stdint.h"


#define EX_UART_NUM             UART_NUM_0
#define PATTERN_CHR_NUM         (3)         /*!< Set the number of consecutive and identical characters received by receiver which defines a UART pattern*/

#define BUF_SIZE                (1024)
#define RD_BUF_SIZE             (BUF_SIZE)

#define ECHO_TEST_TXD           (17)
#define ECHO_TEST_RXD           (16)
#define ECHO_TEST_RTS           (UART_PIN_NO_CHANGE)
#define ECHO_TEST_CTS           (UART_PIN_NO_CHANGE)

#define ECHO_UART_PORT_NUM      (2)
#define ECHO_UART_BAUD_RATE     (115200)
#define ECHO_TASK_STACK_SIZE    (2048)


void uart2_send(char *data, int len);
int uart2_recv();
void create_uart_task(void);
void uart0_init(int baud);
void uart2_init(int baud);

extern uint8_t uart2_rx_flag;
extern uint8_t UART2_RX_BUF[RD_BUF_SIZE]; 

#endif

