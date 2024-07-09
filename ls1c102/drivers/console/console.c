#include <stdbool.h>
 
#include "termios.h"
#include "ls1c102.h"

#include "bsp.h"

#include "ns16550.h"
#include "console.h"

void console_init(unsigned int baudrate)
{
    struct termios t;

    NS16550_init(ConsolePort, NULL);
    
    t.c_cflag = BAUDRATE_TO_CFLAG(baudrate) | CS8;// 8N1
    NS16550_open(ConsolePort, (void *)&t);
}

char console_getch(void)
{
   return Console_get_char(ConsolePort);
}

void console_putch(char ch)
{
    Console_output_char(ConsolePort, ch);
}

void console_putstr(char *s)
{
    while (*s)
    {
    	console_putch(*s++);
    }
}

void USART_SendData(char *buf_all, int uart_rx_count)
{
    for(int i=0; i<uart_rx_count; i++)
    {
        console_putch(*buf_all);
        buf_all++;
    }
}


