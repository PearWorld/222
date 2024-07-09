#include "bsp.h"

#include <stdlib.h>
#include <stdbool.h>
#include <larchintrin.h>

#include "cpu.h"
#include "termios.h"

#include "ls1c102.h"
#include "ls1c102_irq.h"

#include "ns16550.h"
#include "console.h"
#include "src/GPIO/user_gpio.h"
#include <string.h>// 使用 memset()

extern HW_PMU_t *g_pmu;

//-----------------------------------------------------------------------------

#define NS16550_FIFO_SIZE   16
//#define NS16550_DTR
//#define NS16550_RTS

//-----------------------------------------------------------------------------

/*
 * predefind functions
 */
static int NS16550_write_string_int(UART_t *pUART, char *buf, int len);

//-----------------------------------------------------------------------------
// buffer: ring mode, drop the most oldest data when add
//-----------------------------------------------------------------------------

static void clear_data_buffer(NS16550_buf_t *data)
{
    data->Count = 0;
    data->pHead = data->pTail = data->Buf;
}

static int enqueue_to_buffer(NS16550_buf_t *data, char *buf, int len, bool overWrite)
{
    int i;

    for (i=0; i<len; i++)
    {
        *data->pTail = buf[i];
        data->Count++;
        data->pTail++;
        if (data->pTail >= data->Buf + UART_BUF_SIZE)
            data->pTail = data->Buf;
    }

    /*
     * if overflow, override the lastest data
     */
    if (data->Count > UART_BUF_SIZE)    // overflow
    {
        data->Count = UART_BUF_SIZE;
        data->pHead = data->pTail;
    }

    return len;
}

static int dequeue_from_buffer(NS16550_buf_t *data, char *buf, int len)
{
    int i, count;

    count = len < data->Count ? len : data->Count;

    for (i=0; i<count; i++)
    {
        buf[i] = *data->pHead;
        data->Count--;
        data->pHead++;
        if (data->pHead >= data->Buf + UART_BUF_SIZE)
            data->pHead = data->Buf;
    }

    return count;
}

/******************************************************************************
 * NS16550 Interrupt.
 */
static void ls1c102_enable_uart_irq(unsigned int ns16550_base)// 使能串口中断
{
    if (ns16550_base == LS1C102_UART0_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->en |= INTC_UART0;
    }
    else if (ns16550_base == LS1C102_UART1_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->en |= INTC_UART1;
    }
    else if (ns16550_base == LS1C102_UART2_BASE)
    {
        HW_PMU_t *pmu = (HW_PMU_t *)LS1C102_PMU_BASE;
        pmu->CmdSts |= CMDSR_INTEN_UART2;
    }
}

static void ls1c102_disable_uart_irq(unsigned int ns16550_base)// 失能串口中断
{
    if (ns16550_base == LS1C102_UART0_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->en &= ~INTC_UART0;
    }
    else if (ns16550_base == LS1C102_UART1_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->en &= ~INTC_UART1;
    }
    else if (ns16550_base == LS1C102_UART2_BASE)
    {
        HW_PMU_t *pmu = (HW_PMU_t *)LS1C102_PMU_BASE;
        pmu->CmdSts &= ~CMDSR_INTEN_UART2;
    }
}

static void ls1c102_clear_uart_irq(unsigned int ns16550_base)// 清除串口中断状态
{
    if (ns16550_base == LS1C102_UART0_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->clr |= INTC_UART0;
    }
    else if (ns16550_base == LS1C102_UART1_BASE)
    {
        HW_INTC_t *intc = (HW_INTC_t *)LS1C102_INTC_BASE;
        intc->clr |= INTC_UART1;
    }
    else if (ns16550_base == LS1C102_UART2_BASE)
    {
        HW_PMU_t *pmu = (HW_PMU_t *)LS1C102_PMU_BASE;
        pmu->CommandW |= CMDSR_INTSRC_UART2;
    }
}

static void NS16550_interrupt_process(UART_t *pUART)
{
    int i = 0;
    int j = 0;
    char buf[NS16550_FIFO_SIZE];
    int uart_rx_len = 128;
    char buf_all[uart_rx_len];
    memset(buf, '\0', 16);
    memset(buf_all, '\0', 128);
    int uart_rx_count = 0;
    int uart_rx_count0 = 0;
    
    /*
     * Iterate until no more interrupts are pending.
     * 迭代直到不再有中断挂起。每个中断最多接收16个字节的数据，也就是16个字符。
     */
    do
    {
        /* Fetch received characters */
        for (i=0; i<NS16550_FIFO_SIZE; ++i)// FIFO 最多保存16个字节的数据，所以 for 循环最多16次
        {
            if (pUART->hwUART->lsr & NS16550_LSR_DR)
            {
                buf[i] = (char)pUART->hwUART->R0.dat;// R0.dat 只能存放一个字节，一次取一个字节，刚好赋值给 buf[i] 。
                // 多余的数据还处于 FIFO 中，每次取完 R0.dat ， FIFO 中的数据就自动传入 R0.dat 。
                if(buf[i] != 0x0d && buf[i] != 0x0a)
                {
                    buf_all[uart_rx_count] = buf[i];
                    uart_rx_count++;
                }
            }
            else
                break;
        }

        /*
         * Enqueue fetched characters // 对提取的字符排队，存放到 rx_buf 中
         */
        enqueue_to_buffer(&pUART->rx_buf, buf, i, true);
        i = 0;

        /* Check if we can dequeue transmitted characters */ // 检查我们是否可以将传输的字符出列，正常不会执行这个 if 里面的。
        if ((pUART->tx_buf.Count > 0) && (pUART->hwUART->lsr & NS16550_LSR_TFE))
        {
            /*
             * Dequeue transmitted characters // 取消传输字符的队列
             */
            i = dequeue_from_buffer(&pUART->tx_buf, buf, NS16550_FIFO_SIZE);

            /* transmit data with interrupt */ // 通过中断传输数据
            NS16550_write_string_int(pUART, buf, i);
        }

        if (i > 0) // 一般 i = pUART->tx_buf.Count > 0
            pUART->hwUART->R1.ier = NS16550_IER_TX | NS16550_IER_RX;
        else
            pUART->hwUART->R1.ier = NS16550_IER_RX;

        j++;
        memset(buf, '\0', 16);
        if(j == 1) uart_rx_count0 = uart_rx_count;
        
    } while (!(pUART->hwUART->R2.isr & NS16550_ISR_PENDING));
    //还有未处理的中断，就继续循环。一般字节数不大于16，所以一般不循环，只触发一次中断即可。
    
    /* test */
    //char *str0 = "abc";// 不推荐，char 是变量，但是 "abc" 是字符串常量，不能直接用字符串常量给 变量赋值。
    const char *str0 = "abc";// right
    //char str0[] = "abc";// right
    int ret = 0;
    ret = strncmp(buf_all, str0, 3);
    printk("ret = %d\r\n", ret);
    if(ret == 0)
    {
        printk("strncmp buf_all = %s\r\n", buf_all);
    }
    printk("buf = %s\r\n", buf);
    USART_SendData(buf_all, uart_rx_count);
    memset(buf_all, '\0', 128);
    printk("my_memset j = %d, buf_all = %s\r\n", j, buf_all);
    printk("uart_rx_count = %d, uart_rx_count0 = %d\r\n", uart_rx_count, uart_rx_count0);
    /* test */
}

static void NS16550_interrupt_handler(int vector, void *arg)// 中断处理函数
{
    UART_t *pUART = (UART_t *)arg;
        
    if (NULL != pUART)
    {
        ls1c102_disable_uart_irq((unsigned int)pUART->hwUART);  /* 关中断 */
        
        ls1c102_clear_uart_irq((unsigned int)pUART->hwUART);    /* 清中断 */

        NS16550_interrupt_process(pUART);                       /* 中断处理 */

        ls1c102_enable_uart_irq((unsigned int)pUART->hwUART);   /* 开中断 */
    }
}

/******************************************************************************
 * These routines provide control of the RTS and DTR lines
 ******************************************************************************/

#ifdef NS16550_RTS
/*
 * NS16550_assert_RTS
 */
static int NS16550_assert_RTS(UART_t *pUART)
{
    loongarch_critical_enter();
    pUART->hwUART->
    pUART->ModemCtrl |= SP_MODEM_RTS;
    NS16550_set_r(pUART->CtrlPort, NS16550_MODEM_CONTROL, pUART->ModemCtrl);
    loongarch_critical_exit();
    return 0;
}

/*
 * NS16550_negate_RTS
 */
static int NS16550_negate_RTS(UART_t *pUART)
{
    loongarch_critical_enter();
    pUART->ModemCtrl &= ~SP_MODEM_RTS;
    NS16550_set_r(pUART->CtrlPort, NS16550_MODEM_CONTROL, pUART->ModemCtrl);
    loongarch_critical_exit();
    return 0;
}
#endif

/******************************************************************************
 * These flow control routines utilise a connection from the local DTR
 * line to the remote CTS line
 ******************************************************************************/

#ifdef NS16550_DTR
/*
 * NS16550_assert_DTR
 */
static int NS16550_assert_DTR(UART_t *pUART)
{
    loongarch_critical_enter();
    pUART->ModemCtrl |= SP_MODEM_DTR;
    NS16550_set_r(pUART->CtrlPort, NS16550_MODEM_CONTROL, pUART->ModemCtrl);
    loongarch_critical_exit();
    return 0;
}

/*
 * NS16550_negate_DTR
 */
static int NS16550_negate_DTR(UART_t *pUART)
{
    loongarch_critical_enter();
    pUART->ModemCtrl &= ~SP_MODEM_DTR;
    NS16550_set_r(pUART->CtrlPort, NS16550_MODEM_CONTROL, pUART->ModemCtrl);
    loongarch_critical_exit();
    return 0;
}

#endif

/******************************************************************************
 * NS16550_set_attributes
 */
static int NS16550_set_attributes(UART_t *pUART, const struct termios *t)
{
    unsigned int divisor, decimal, baud_requested, irq_mask;
    unsigned char lcr = 0, win_size, sample_point;

    if (NULL == t)
        return -1;

    /* Calculate the baud rate divisor */
    baud_requested = t->c_cflag & CBAUD;
    if (!baud_requested)
        baud_requested = B115200;                   /* default to 115200 baud */
    baud_requested = CFLAG_TO_BAUDRATE(baud_requested);
    
    win_size     = pUART->hwUART->samp & 0x0F;      /* 1bit 划分采样点数 */
    sample_point = pUART->hwUART->samp >> 4;        /* 采样位置 */
    if ((win_size == 0) || (sample_point == 0))
    {
        pUART->hwUART->samp = 0x38;
        win_size = 8;
    }

#if 0
    divisor = BUS_FREQUENCY / baud_requested / win_size;
    decimal = BUS_FREQUENCY - (divisor * baud_requested * win_size);
#else
    divisor = bus_frequency / baud_requested / win_size;
    decimal = bus_frequency - (divisor * baud_requested * win_size);
#endif
    decimal = (decimal * 255) / baud_requested / win_size;

    /* Parity */
    if (t->c_cflag & PARENB)
    {
        lcr |= NS16550_LCR_PE;
        if (!(t->c_cflag & PARODD))
            lcr |= NS16550_LCR_EPS;
    }

    /* Character Size */
    if (t->c_cflag & CSIZE)
    {
        switch (t->c_cflag & CSIZE)
        {
            case CS5:  lcr |= NS16550_LCR_BITS_5; break;
            case CS6:  lcr |= NS16550_LCR_BITS_6; break;
            case CS7:  lcr |= NS16550_LCR_BITS_7; break;
            case CS8:  lcr |= NS16550_LCR_BITS_8; break;
        }
    }
    else
    {
        lcr |= NS16550_LCR_BITS_8;         /* default to 9600,8,N,1 */
    }

    /* Stop Bits */
    if (t->c_cflag & CSTOPB)
        lcr |= NS16550_LCR_SB;             /* 2 stop bits */

    /*
     * Now actually set the chip
     */
    loongarch_critical_enter();

    /* Save port interrupt mask */
    irq_mask = pUART->hwUART->R1.ier;

    /* Set the baud rate */
    pUART->hwUART->lcr = NS16550_LCR_DLAB;

    /* XXX are these registers right? */
    pUART->hwUART->R0.dll = divisor & 0xFF;
    pUART->hwUART->R1.dlh = (divisor >> 8) & 0xFF;
    pUART->hwUART->R2.dld = decimal & 0xFF;

    /* Now write the line control */
    pUART->hwUART->lcr = lcr;

    /* Restore port interrupt mask */
    pUART->hwUART->R1.ier = irq_mask;

    READ_REG8(&pUART->hwUART->lsr);
    READ_REG8(&pUART->hwUART->R0.dat);
#ifdef NS16550_DTR
    READ_REG8(&pUART->hwUART->msr);
#endif

    loongarch_critical_exit();

    return 0;
}

/******************************************************************************
 * NS16550_init
 */
int NS16550_init(void *dev, void *arg)
{
    UART_t *pUART = (UART_t *)dev;

    if (NULL == dev)
        return -1;

#if (BSP_USE_UART0)
    if (dev == devUART0)
    {
        g_pmu->IOSEL0 |= ((1<<12)|(1<<14));  // GPIO 6~7
    }
#endif
#if (BSP_USE_UART1)
    else if (dev == devUART1)
    {
        g_pmu->IOSEL0 |= ((1<<16)|(1<<18));  // GPIO 8~9
    }
#endif
#if (BSP_USE_UART2)
    else if (dev == devUART2)
    {
        g_pmu->IOSEL0 |= ((1<<20)|(1<<22));  // GPIO 10~11
    }
#endif
    else
    {
        return -1;
    }

    if (NULL != arg)
    {
        struct termios t;
        unsigned int baud = *(unsigned int *)arg;
        t.c_cflag = BAUDRATE_TO_CFLAG(baud) | CS8;
        NS16550_set_attributes(pUART, &t);
    }
    else
    {
        pUART->hwUART->lcr = 0;
        pUART->hwUART->R1.ier = 0;
    }
    
    /* Enable and reset transmit and receive FIFOs. */
    pUART->hwUART->R2.fcr = NS16550_FCR_FIFO_EN |
                            NS16550_FCR_TXFIFO_RST |
                            NS16550_FCR_RXFIFO_RST |
                            NS16550_FCR_TRIGGER(1);// 设置触发中断需要接收的字节数

#ifdef NS16550_DTR
    /* Set data terminal ready. */
    /* And open interrupt tristate line */
    pUART->hwUART->mcr = SP_MODEM_IRQ;
    READ_REG8(&pUART->hwUART->msr);
#endif

    READ_REG8(&pUART->hwUART->lsr);
    READ_REG8(&pUART->hwUART->R0.dat);

    /* Eable Console Port Interrupt Mode */
    if (pUART == ConsolePort)
        pUART->bInterrupt = true;

    return 0;
}

/******************************************************************************
 * NS16550_open
 */
int NS16550_open(void *dev, void *arg)
{
    UART_t *pUART = (UART_t *)dev;
    struct termios *t = (struct termios *)arg;

    if (NULL == dev)
        return -1;

    clear_data_buffer(&pUART->rx_buf);
    clear_data_buffer(&pUART->tx_buf);

#ifdef NS16550_DTR
    /* Assert DTR */
    if (pUART->bFlowCtrl)
        NS16550_assert_DTR(pUART);
#endif

    pUART->hwUART->R1.ier = 0;
    
    /* Set initial baud */
    if (NULL != t)
    {
        NS16550_set_attributes(pUART, t);
    }

    if (pUART->bInterrupt)
    {
        /**
         * 安装中断向量
         */
        ls1c102_install_isr(pUART->irqNum,
                            NS16550_interrupt_handler,
                            dev);
        
        ls1c102_enable_uart_irq((unsigned int)pUART->hwUART);/* 开中断 */
        pUART->hwUART->R1.ier = NS16550_IER_RX;// 接收状态中断使能
    }

    return 0;
}

/******************************************************************************
 * NS16550_close
 */
int NS16550_close(void *dev, void *arg)
{
    UART_t *pUART = (UART_t *)dev;

    if (NULL == dev)
        return -1;
        
#ifdef NS16550_DTR
    /* Negate DTR */
    if (pUART->bFlowCtrl)
        NS16550_negate_DTR(pUART);
#endif

    pUART->hwUART->R1.ier = 0;
    if (pUART->bInterrupt)
    {
        ls1c102_disable_uart_irq((unsigned int)pUART->hwUART);
        
        ls1c102_remove_isr(pUART->irqNum);          /* 移除中断向量 */
    }

    return 0;
}

/*
 * Polled write for NS16550.
 */
static void NS16550_output_char_polled(UART_t *pUART, char ch)
{
    /* Save port interrupt mask */
    unsigned char irq_mask = pUART->hwUART->R1.ier;

    /* Disable port interrupts */
    pUART->hwUART->R1.ier = 0;

    while (true)
    {
        /* Try to transmit the character in a critical section */
        loongarch_critical_enter();

        /* Read the transmitter holding register and check it */
        if (pUART->hwUART->lsr & NS16550_LSR_TFE)
        {
            /* Transmit character */
            pUART->hwUART->R0.dat = ch;
            /* Finished */
            loongarch_critical_exit();
            break;
        }
        else
        {
            loongarch_critical_exit();
        }

        /* Wait for transmitter holding register to be empty
         * FIXME add timeout about 2ms, one byte transfer at 4800bps
         */
        while (!(pUART->hwUART->lsr & NS16550_LSR_TFE))
            ;
    }

    /* Restore port interrupt mask */
    pUART->hwUART->R1.ier = irq_mask;
}

/*
 * Interrupt write for NS16550.
 */
static int NS16550_write_string_int(UART_t *pUART, char *buf, int len)
{
    int i, out = 0;

    if (pUART->hwUART->lsr & NS16550_LSR_TFE)// 传输 FIFO 为空后满足 if
    {
        out = len > NS16550_FIFO_SIZE ? NS16550_FIFO_SIZE : len;
        for (i=0; i<out; ++i)
        {
            pUART->hwUART->R0.dat = buf[i];
        }

        if (out > 0)// 好像一定满足 if
            pUART->hwUART->R1.ier = NS16550_IER_TX | NS16550_IER_RX;// 发送状态中断使能、接收状态中断使能
        else
            pUART->hwUART->R1.ier = NS16550_IER_RX;// 接收状态中断使能
    }

    /*
     * remain bytes to buffer // 将剩余字节的数据排到 tx_buf 后面。
     */
    if (len > out)
    {
        loongarch_critical_enter();
        out += enqueue_to_buffer(&pUART->tx_buf, buf + out, len - out, false);
        loongarch_critical_exit();
    }

    return out;
}

/*
 * Polled write string
 */
static int NS16550_write_string_polled(UART_t *pUART, char *buf, int len)
{
    int nwrite = 0;

    /*
     * poll each byte in the string out of the port.
     */
    while (nwrite < len)
    {
        /* transmit character */
        NS16550_output_char_polled(pUART, *buf++);
        nwrite++;
    }

    /* return the number of bytes written. */
    return nwrite;
}

/*
 * Polled get char nonblocking
 */
static int NS16550_inbyte_nonblocking_polled(UART_t *pUART)
{
    unsigned char ch;

    if (pUART->hwUART->lsr & NS16550_LSR_DR)// FIFO 接收到数据后满足 if
    {
        ch = pUART->hwUART->R0.dat;
        return (int)ch;
    }

    return -1;
}

/*
 * Polled get char blocking
 */
static int NS16550_inbyte_blocking_polled(UART_t *pUART)
{
    unsigned char ch = -1;

    for ( ; ; )
    {
        if (pUART->hwUART->lsr & NS16550_LSR_DR)
        {
            ch = pUART->hwUART->R0.dat;
            break;
        }

        delay_ms(1);            /* wait 1 ms */
    }

    return (int)ch;
}

/******************************************************************************
 * NS16550 read
 */
int NS16550_read(void *dev, unsigned char *buf, int size, void *arg)
{
    UART_t *pUART = (UART_t *)dev;
    int blocking, rdBytes;
    
    if (NULL == dev)
        return -1;

    if (NULL == arg)
        blocking = 0;
    else
        blocking = (int)arg;

    if (pUART->bInterrupt)
    {
        /* read from buffer */
        loongarch_critical_enter();
        rdBytes = dequeue_from_buffer(&pUART->rx_buf, (char *)buf, size);
        loongarch_critical_exit();

        return rdBytes;
    }
    else if (!blocking)
    {
        int i=0, val;
        for (i=0; i<size; i++)
        {
            val = NS16550_inbyte_nonblocking_polled(pUART);
            buf[i] = (unsigned char)val;
            if (val == -1)
                return i;
        }

        return size;
    }
    else
    {
        int i=0, val;
        for (i=0; i<size; i++)
        {
            val = NS16550_inbyte_blocking_polled(pUART);
            buf[i] = (unsigned char)val;
        }

        return size;
    }
}

/******************************************************************************
 * NS16550 write
 */
int NS16550_write(void *dev, unsigned char *buf, int size, void *arg)
{
    UART_t *pUART = (UART_t *)dev;

    if (NULL == dev)
        return -1;

    if (pUART->bInterrupt)
    {
        return NS16550_write_string_int(pUART, (char *)buf, size);
    }
    else
    {
        return NS16550_write_string_polled(pUART, (char *)buf, size);
    }
}

/******************************************************************************
 * NS16550 control
 */
int NS16550_ioctl(void *dev, unsigned cmd, void *arg)
{
    UART_t *pUART = (UART_t *)dev;

    if (NULL == dev)
        return -1;

    switch (cmd)
    {
        case IOC_NS16550_SET_MODE:
            /* Set initial baud */
            NS16550_set_attributes(pUART, (struct termios *)arg);
            break;

        default:
            break;
    }

    return 0;
}

//-----------------------------------------------------------------------------
// Console Support
//-----------------------------------------------------------------------------

char Console_get_char(UART_t *pUART)
{
    return (char)NS16550_inbyte_nonblocking_polled(pUART);
}

void Console_output_char(UART_t *pUART, char ch)
{
    NS16550_output_char_polled(pUART, ch);
}

//-----------------------------------------------------------------------------
// UART devices, 必须放在 bss 段
//-----------------------------------------------------------------------------

/* UART 0 */
#if (BSP_USE_UART0)
static UART_t ls1c_UART0 =
{
    .hwUART     = (HW_NS16550_t *)LS1C102_UART0_BASE,
    .irqNum     = LS1C102_IRQ_UART0,
    .bInterrupt = false,
    .baudRate   = 115200,
};
UART_t *devUART0 = &ls1c_UART0;
#endif

/* UART 1 */
#if (BSP_USE_UART1)
static UART_t ls1c_UART1 =
{
    .hwUART     = (HW_NS16550_t *)LS1C102_UART1_BASE,
    .irqNum     = LS1C102_IRQ_UART1,
    .bInterrupt = false,
    .baudRate   = 115200,
};
UART_t *devUART1 = &ls1c_UART1;
#endif

/* UART 2 */
#if (BSP_USE_UART2)
static UART_t ls1c_UART2 =
{
    .hwUART     = (HW_NS16550_t *)LS1C102_UART2_BASE,
    .irqNum     = LS1C102_IRQ_UART2,
    .bInterrupt = false,
    .baudRate   = 115200,
};
UART_t *devUART2 = &ls1c_UART2;
#endif

//-----------------------------------------------------------------------------
// UART as Console
//-----------------------------------------------------------------------------

UART_t *ConsolePort = &ls1c_UART1;

//-----------------------------------------------------------------------------

/*
 * @@ END
 */
