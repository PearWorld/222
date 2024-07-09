#ifndef __CONSOLE_H__
#define __CONSOLE_H__

#ifdef __cplusplus
extern "C" {
#endif

void console_init(unsigned int baudrate);
char console_getch(void);
void console_putch(char ch);
void console_putstr(char *s);

void USART_SendData(char *buf_all, int uart_rx_count);

#ifdef __cplusplus
}
#endif

#endif /*__CONSOLE_H__*/

