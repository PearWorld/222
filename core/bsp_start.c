#include <string.h>
#include <larchintrin.h>

#include "bsp.h"
#include "cpu.h"

#include "ls1c102.h"
#include "clock.h"

//-----------------------------------------------------------------------------

extern void exception_common_entry(void);
extern void console_init(unsigned int baudrate);
extern int main(void);

//-----------------------------------------------------------------------------

/*
 * global Variable
 */
unsigned int cpu_frequency;     // CPU ����Ƶ��
unsigned int bus_frequency;     // BUS ����Ƶ��

/******************************************************************************
 * ȫ�ֿ����� 
 */
HW_PMU_t *g_pmu = (HW_PMU_t *)LS1C102_PMU_BASE;

#if 1
/*
 * ʹ���ⲿ 8M ʱ������
 */
static void OSC_init_outer(void)
{
    g_pmu->ChipCtrl &= ~CHIPCTRL_8M_SEL;
    g_pmu->ChipCtrl |= CHIPCTRL_8M_EN;

    //TODO;     /* ������ʱ��˯���������ʱ��ʧЧ? ��loop ����ѭ�� */

    while (g_pmu->CmdSts & CMDSR_8M_FAIL)
    {
        g_pmu->ChipCtrl &= ~CHIPCTRL_8M_SEL;
    }

    g_pmu->ChipCtrl |= CHIPCTRL_8M_SEL;
}
#endif

static void get_frequency(void)
{
    unsigned int val = g_pmu->CmdSts;

    if ((val & CMDSR_8M_SEL) && !(val & CMDSR_8M_FAIL)) /* ʹ���ⲿ 8M ����*/
    {
        cpu_frequency = 8000000;
        bus_frequency = 8000000;
    }
    else
    {
#if 0
        /*
         * ������þ���Ƶ���ڼĴ��� 0xBF0201B0 ��?
         */
        cpu_frequency = (*(volatile unsigned int *)0xBF0201B0) * 1000;

        if (cpu_frequency > 0)
        {
            bus_frequency = cpu_frequency;
            return;
        }
#endif

        /*
         * �ڲ� 32MHZ ����
         */
        val = g_pmu->ChipCtrl;
        
        if (val & CHIPCTRL_FASTEN)      // 3 ��Ƶ
        {
            cpu_frequency = 32000000 / 3;
            bus_frequency = 32000000 / 3;
        }
        else                            // 4 ��Ƶ
        {
            cpu_frequency = 8000000;
            bus_frequency = 8000000;
        }
    }
}

/****************************************************************************** 
 * ls1c102 bsp start
 */

extern void ls1c102_init_isr_table(void);

void bsp_start(void)
{
    unsigned int eentry;

    loongarch_interrupt_disable();

    /**
     * ���жϴ��븴�Ƶ�0x80000000
     */
    eentry = __csrrd_w(LA_CSR_EBASE);
    memcpy((void *)eentry, (void *)exception_common_entry, 32);

    g_pmu->CommandW = 0;            // CMDSR_SLEEP_EN;

    OSC_init_outer();               // ����Ϊ�ⲿ8Mʱ�ӣ� TODO delay_ms() �����С�
    // ��ʹ���ⲿ 8M ʱ�ӻᵼ�´��ڴ�ӡ���룬��Ϊ�ڲ�ʱ�ӵ�Ƶ�ʲ���ȷ���Ͳ����ʲ�ƥ�䡣
    
    get_frequency();                /* ��ȡƵ������ */

    ls1c102_init_isr_table();       /* ��ʼ���ж������� */

    console_init(115200);           /* initialize Console */

    Clock_initialize();             /* initialize system ticker */

    loongarch_interrupt_enable();   /* Enable all interrupts */

    main();                         /* goto main function */
    
    while (1);                      /* XXX never goto HERE! */
    
    return;
}


