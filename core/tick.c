#include <stdio.h>
#include <stdint.h>
#include <larchintrin.h>

#include "cpu.h"
#include "regdef.h"

#include "ls1c102.h"
#include "ls1c102_irq.h"

//-----------------------------------------------------------------------------

#define TICKS_PER_SECOND    1000

extern unsigned int cpu_frequency;

static volatile unsigned int delay_1us_count;       /* hda 延时1us的计数值 */

static volatile unsigned int Clock_driver_ticks;    /* Clock ticks since initialization */

//-----------------------------------------------------------------------------

unsigned int get_clock_ticks(void)
{
    return Clock_driver_ticks;
}

/*
 *  Clock_isr
 *
 *  This is the clock tick interrupt handler.
 */
void ls1c102_ticker_isr(int vector, void *arg)
{
    ++Clock_driver_ticks;           /* 计数加 1 */
}

/*
 * Clock_initialize
 */
void Clock_initialize(void)
{
    unsigned int tcfg;
    
    Clock_driver_ticks = 0;

    __csrwr_w(0, LA_CSR_TVAL);
    __csrwr_w(0, LA_CSR_CNTC);

    /**
     * 安装中断向量
     */
    ls1c102_install_isr(LS1C102_IRQ_TICKER,
                        ls1c102_ticker_isr,
                        NULL);

    tcfg = cpu_frequency / TICKS_PER_SECOND;
    tcfg <<= CSR_TCFG_VAL_SHIFT;
    tcfg |= CSR_TCFG_PERIOD | CSR_TCFG_EN;

    __csrwr_w(tcfg, LA_CSR_TCFG);

    printk("\nClock: %i per second\n", TICKS_PER_SECOND);

    delay_1us_count = cpu_frequency / 1000000;
}

/*
 * TODO 限制 us
 */
void delay_us(unsigned int us)
{
    // new_method
	#if 1
		/*
		* 用循环来实现 us 级延时
		*/
		register unsigned int i, count;
		int delay_1us_insns = 1;

	#if 1
		//count = us * delay_1us_insns / 32;
		count = us * delay_1us_insns / 64;
	#else
		count = us * (cpu_frequency / 1000) / 4000;
	#endif

		for (i = 0; i < count; i++)
		{
			asm volatile( "nop ; ");
		}
	#else
	#endif
	// new_method
}

void delay_ms(unsigned int ms)
{
    register unsigned int startTicks, endTicks, curTicks;

    /*
     * 如果关中断, 调用 delay_us 延时
     */
    if ((__csrrd_w(LA_CSR_CRMD) & CSR_CRMD_IE) == 0)
    {
        delay_us(ms * 1000);
        return;
    }

    startTicks = Clock_driver_ticks;
    endTicks   = startTicks + ms * TICKS_PER_SECOND / 1000;

    while (1)
    {
        curTicks = Clock_driver_ticks;

        /*
         * 防止数值溢出
         */
        if (((endTicks > startTicks) && (curTicks >= endTicks)) ||
            ((endTicks < startTicks) && (curTicks < startTicks) && (curTicks >= endTicks)))
            break;
    }
}


