#include <stdio.h>
#include <stdbool.h>

#include "bsp.h"
#include "console.h"
#include "ls1c102.h"

#include "src/GPIO/user_gpio.h"
#include "src/WDG/ls1x_wdg.h"

extern HW_PMU_t *g_pmu;

int main(void)
{
    OLED_Init();// 初始化OLED
	OLED_Clear();// 清屏（全黑）	
    OLED_ShowChar(0, 0, 'lsin', 16);
    gpio_init(1, 0);// GPIO1 使能输入
    gpio_init(2, 0);
    gpio_init(22, 1);// GPIO22 使能输出
    gpio_init(23, 1);
    gpio_write(23, 1);
    gpio_write(22, 0);
    WdgInit();
    printk("main() function\r\n");
    gpio_init(20, 1);// GPIO20 使能输出
    console0_init(115200);// 串口0初始化
    gpio_iosel(6, 1);
    gpio_iosel(7, 1);
    console0_init(115200);// 串口0初始化
    esp8266_start_trans();		 // esp8266进行初始化
	esp8266_send_data("1234");// 通过 esp8266 向服务端（电脑或者手机的网络调试助手）发送数据1234
	const char *str0 = "on";// right
    const char *str1 = "off";// right
    char string0[100] = "A";
    char *string1 = "abc";

int long fftin [NPT];//FFT输入
int long fftout[NPT];//FFT输出
u32 FFT_Mag[NPT/2]={0};//幅频特性
u16 magout[NPT];//模拟正弦波输出缓存区

u16 table[15] ={16,32,48,64,80,96,112,128,144,160,176,192,208,224,240};//标点横坐标

u16 currentadc;//实时采样数据
u16 adcx[NPT];//adc数值缓存
u32 adcmax;//1024点中adc最大值
u32 adcmin;//1024点中adc最小值
float ADCMAX;//显示两位小数max(mv)
float ADCMIN;//显示两位小数min(mv)
float ADCCZ;//显示两位小数Vpp（峰峰值）
char ADCMax[20];//char类型字符串显示
char ADCMin[20];//char类型字符串显示
char ADCcz[20];//char类型字符串显示
u8 adc_flag=0;//采样结束标志
u8 key_flag=0;//按键扫描标志
u8 show_flag=1;//更新暂停标志
u32 fre;//采样频率 Hz
u32 FRE;//采样频率 KHz

u8 f1;//信号1频率 
//char f1[20];
u8 f2;//信号2频率 
//char f2[20];
u8 fm1;//dds输入频率（KHz）
u8 fm2;//dds输出频率（KHz）
u16 V=165;//纵坐标单位刻度 mv/div
u16 t=0;//adc采样下标
u16 key;//按键值 
u16 temp=0;//幅值最大的频率成分 （实际信号的最大频率所对应的数）
float F;//波形频率  
u8 start_flag;
/*********************************分离有关变量*****************************/

//2 144(250KHz) 10 16(450KHz)
u16 T=10;//定时器2重载值（ARR），不能小于PWM的Pluse值（CCR）
s16 pre=16;//定时器2预分频值（PSC） 

u16 point[17];//20KHz 25Khz 30KHz...采样点数

u8 cy[]={20,25,30,35,40,45,50,55,60,65,70,75,80,85,90,95,100}; 
//20KHz 25Khz 30KHz...波的频率

u8 f1_point;//fA波在频谱中频率的点数
u8 f2_point;//fB波在频谱中频率的点数

/********************************不漂移有关变量****************************/
u8 ht_flag=0;//绘波形图
u8 ddgdp_flag=0;//等待电平最高点
u16 ADCmax[15];//采集15个1024去更新dds来减少漂移
u8 cycs;//采样次数
/********************************相位改变有关变量**************************/
uint16_t Phase;//A撇和B撇的相位差

void BGInit(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_15;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
}

void BJ_Loop(void)
{
	start_flag = 1;
	GPIO_ResetBits(GPIOB, GPIO_Pin_14);
	GPIO_ResetBits(GPIOB, GPIO_Pin_15);
	if(F<20)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_14);
		start_flag = 0;
	}
	if(F>100)
	{
		GPIO_SetBits(GPIOB, GPIO_Pin_15);
		start_flag = 0;
	}
}

    for (;;)
    {

        if(gpio_read(1)&&!gpio_read(2))
        {
        gpio_write(22, 1);
        gpio_write(23, 0);
        delay_ms(100);
        gpio_write(23, 1);
        }
        else if(!gpio_read(1)&&gpio_read(2))
        {
        gpio_write(23, 0);
        delay_ms(100);
        gpio_write(23, 1);

        gpio_write(22, 1);
        delay_ms(500);
        gpio_write(22, 0);
        }
        else
        {
        gpio_write(23, 1);
        gpio_write(22, 0);
        }
                register unsigned int ticks;
        ticks = get_clock_ticks();
        printk("tick count = %u\r\n", ticks);
        printk("abcdefghijklmnopqrstuvwxyz\r\n");
        gpio_write(20, 1);// GPIO20 输出高电平
        delay_ms(100);
        gpio_write(20, 0);// GPIO20 输出低电平
        delay_ms(300);

        console_putstr(string0, 1);
        printk("\r\n");
        console_putstr(string1, 1);
        printk("\r\n");
        usart0_print("string1 = %s\r\n", string1);
         int ret = 0;
        ret = strncmp((char *)UART0_RX_BUF, (char *)str0, 2);
        if(ret == 0)
        {
            gpio_write(13, 1);
            printk("led on\r\n");
        }
        ret = strncmp((char *)UART0_RX_BUF, (char *)str1, 3);
        if(ret == 0)
        {
            gpio_write(13, 0);
            printk("led off\r\n");
        }
        delay_ms(2);
    }

    /*
     * Never goto here!
     */
    return 0;
}

/*
 * @@ End
 */
/******************************************************************
函数名称:GetPowerMag()
函数功能:计算各次谐波幅值 
参数说明:
备　　注:先将lBufOutArray分解成实部(X)和虚部(Y)，然后计算模值(sqrt(X*X+Y*Y)
*******************************************************************/
void GetPowerMag(void)
{
	float X, Y, Mag, magmax;//实部，虚部，各频率相位
	signed short lX,lY; //设置这两个变量为了符合浮点数运算
	u16 i;

	//调用自cr4_fft_1024_stm32
	cr4_fft_1024_stm32(fftout, fftin, NPT);	
	

	for(i=1; i<NPT/2; i++)
	{
		lX  = (fftout[i] << 16) >> 16;
		lY  = (fftout[i] >> 16);
		X = NPT * ((float)lX) / 32768;  //低16位存实部
		Y = NPT * ((float)lY) / 32768;	//高16位存虚部	
		Mag = sqrt(X * X + Y * Y)/NPT;//计算模值
		if(Mag > magmax)
		{
			magmax = Mag;
			temp = i;
		}
		if(i == 0)   //算出幅值
		{
			FFT_Mag[i] = (unsigned long)(Mag * 32768);
		}
		else
		{
			FFT_Mag[i] = (unsigned long)(Mag * 65536);
		}
	}
	
	
	F=(temp*(fre*1.0/NPT))/1000;	//FFT所得实际频率f=点数i*(采样频率F/总采样点数N)
	LCD_ShowNum(600,330,F,3,24);
	BJ_Loop();
	if(start_flag)
	{
		FL_f();  //得到f1 f2频率
		FL_lx(); //得到f1 f2类型
	}
	LCD_ShowNum(100,330,f1,3,24);

	LCD_ShowNum(260,330,f2,3,24);
	
	
}

/******************************************************************
简介：分离得到波的频率
*******************************************************************/	
void FL_f(void)
{
	u16 i,j;
	f1=0;
	f2=0;
	f1_point=0;//fA波的点数
	f2_point=0;//fB波的点数
	for(i=0;i<17;i++)
	{
		for(j=point[i]-2;j<=point[i]+2;j++)
		{
			if(FFT_Mag[j]>200)
			{
				if(!f1)
				{
					f1=cy[i];
					f1_point=point[i];
					fm1=f1;
				}
				else
				{
					f2=cy[i];
					f2_point=point[i];
					fm2=f2;
				}
			}
		}
		
	}
	//取第三次ADC采样的结果来初始化DDS 
	if(cycs==3)
	{
		AD9959_Set_Fre(CH0, fm1*1000);	//设置通道0频率100000Hz
		AD9959_Set_Fre(CH1, fm2*1000);	//设置通道1频率100000Hz
		AD9959_Set_Fre(CH2, 100000);	//设置通道2频率100000Hz
		AD9959_Set_Fre(CH3, 100000);	//设置通道3频率100000Hz
			
		AD9959_Set_Amp(CH0, 1023); 		//设置通道0幅度控制值1023，范围0~1023
		AD9959_Set_Amp(CH1, 1023); 		//设置通道1幅度控制值1023，范围0~1023
		AD9959_Set_Amp(CH2, 1023); 		//设置通道2幅度控制值1023，范围0~1023
		AD9959_Set_Amp(CH3, 1023); 		//设置通道3幅度控制值1023，范围0~1023

		AD9959_Set_Phase(CH0, 0);			//设置通道0相位控制值0(0度)，范围0~16383
		AD9959_Set_Phase(CH1, 0);	//设置通道1相位控制值4096(90度)，范围0~16383
		AD9959_Set_Phase(CH2, 8192);	//设置通道2相位控制值8192(180度)，范围0~16383
		AD9959_Set_Phase(CH3, 12288);	//设置通道3相位控制值12288(270度)，范围0~16383
	}
}

/******************************************************************
简介:判断分离出来的两个波的类型
*******************************************************************/	
void FL_lx(void)
{
	u8 f1_sj=0,f1_sin=0;//fA三角波 正弦波标志位
	u8 f2_sj=0,f2_sin=0;//fB三角波 正弦波标志位
	switch(f1)
	{
		case 20:
		{
			if(FFT_Mag[46]>370)
			f1_sj=0;  //正弦波
			if(FFT_Mag[46]<370)
			f1_sj=1;  //三角波
		}break;
		case 25:
		{
			if(FFT_Mag[57]>560)
			f1_sj=0;
			if(FFT_Mag[57]<560)
			f1_sj=1;
		}break;
		case 30:
		{
			if(FFT_Mag[68]>510)
			f1_sj=0; 
			if(FFT_Mag[68]<510)
			f1_sj=1; 
		}break;
		case 35:
		{
			if(FFT_Mag[80]>460)
			f1_sj=0; 
			if(FFT_Mag[80]<460)
			f1_sj=1; 
		}break;
		case 40:
		{
			if(FFT_Mag[91]>580)
			f1_sj=0; 
			if(FFT_Mag[91]<580)
			f1_sj=1; 
		}break;
		case 45:
		{
			if(FFT_Mag[102]>440)
			f1_sj=0; 
			if(FFT_Mag[102]<440)
			f1_sj=1; 
		}break;
		case 50:
		{
			if(FFT_Mag[114]>530)
			f1_sj=0; 
			if(FFT_Mag[114]<530)
			f1_sj=1; 
		}break;
		case 55:
		{
			if(FFT_Mag[125]>550)
			f1_sj=0; 
			if(FFT_Mag[125]<550)
			f1_sj=1; 
		}break;
		case 60:
		{
			if(FFT_Mag[137]>390)
			f1_sj=0; 
			if(FFT_Mag[137]<390)
			f1_sj=1; 
		}break;
		case 65:
		{
			if(FFT_Mag[148]>570)
			f1_sj=0; 
			if(FFT_Mag[148]<570)
			f1_sj=1; 
		}break;
		case 70:
		{
			if(FFT_Mag[159]>500)
			f1_sj=0; 
			if(FFT_Mag[159]<500)
			f1_sj=1; 
		}break;
		case 75:
		{
			if(FFT_Mag[171]>490)
			f1_sj=0; 
			if(FFT_Mag[171]<490)
			f1_sj=1; 
		}break;
		case 80:
		{
			if(FFT_Mag[182]>570)
			f1_sj=0; 
			if(FFT_Mag[182]<570)
			f1_sj=1; 
		}break;
		case 85:
		{
			if(FFT_Mag[193]>425)
			f1_sj=0; 
			if(FFT_Mag[193]<425)
			f1_sj=1; 
		}break;
		case 90:
		{
			if(FFT_Mag[205]>540)
			f1_sj=0; 
			if(FFT_Mag[205]<540)
			f1_sj=1; 
		}break;
		case 95:
		{
			if(FFT_Mag[216]>550)
			f1_sj=0; 
			if(FFT_Mag[216]<550)
			f1_sj=1; 
		}break;
		case 100:
		{
			if(FFT_Mag[228]>400)
			f1_sj=0; 
			if(FFT_Mag[228]<400)
			f1_sj=1; 
		}break;
	}
	
	switch(f2)
	{
		case 20:
		{
			if(FFT_Mag[46]>370)
			f2_sj=0; //三角波
			if(FFT_Mag[46]<370)
			f2_sj=1; //正弦波
		}break;
		case 25:
		{
			if(FFT_Mag[57]>560)
			f2_sj=0; 
			if(FFT_Mag[57]<560)
			f2_sj=1; 
		}break;
		case 30:
		{
			if(FFT_Mag[68]>510)
			f2_sj=0; 
			if(FFT_Mag[68]<510)
			f2_sj=1; 
		}break;
		case 35:
		{
			if(FFT_Mag[80]>460)
			f2_sj=0; 
			if(FFT_Mag[80]<460)
			f2_sj=1; 
		}break;
		case 40:
		{
			if(FFT_Mag[91]>580)
			f2_sj=0; 
			if(FFT_Mag[91]<580)
			f2_sj=1; 
		}break;
		case 45:
		{
			if(FFT_Mag[102]>440)
			f2_sj=0; 
			if(FFT_Mag[102]<440)
			f2_sj=1; 
		}break;
		case 50:
		{
			if(FFT_Mag[114]>530)
			f2_sj=0; 
			if(FFT_Mag[114]<530)
			f2_sj=1; 
		}break;
		case 55:
		{
			if(FFT_Mag[125]>550)
			f2_sj=0; 
			if(FFT_Mag[125]<550)
			f2_sj=1; 
		}break;
		case 60:
		{
			if(FFT_Mag[137]>390)
			f2_sj=0; 
			if(FFT_Mag[137]<390)
			f2_sj=1; 
		}break;
		case 65:
		{
			if(FFT_Mag[148]>570)
			f2_sj=0; 
			if(FFT_Mag[148]<570)
			f2_sj=1; 
		}break;
		case 70:
		{
			if(FFT_Mag[159]>500)
			f2_sj=0; 
			if(FFT_Mag[159]<500)
			f2_sj=1; 
		}break;
		case 75:
		{
			if(FFT_Mag[171]>490)
			f2_sj=0; 
			if(FFT_Mag[171]<490)
			f2_sj=1; 
		}break;
		case 80:
		{
			if(FFT_Mag[182]>570)
			f2_sj=0; 
			if(FFT_Mag[182]<570)
			f2_sj=1; 
		}break;
		case 85:
		{
			if(FFT_Mag[193]>425)
			f2_sj=0; 
			if(FFT_Mag[193]<425)
			f2_sj=1; 
		}break;
		case 90:
		{
			if(FFT_Mag[205]>540)
			f2_sj=0; 
			if(FFT_Mag[205]<540)
			f2_sj=1; 
		}break;
		case 95:
		{
			if(FFT_Mag[216]>550)
			f2_sj=0; 
			if(FFT_Mag[216]<550)
			f2_sj=1; 
		}break;
		case 100:
		{
			if(FFT_Mag[228]>400)
			f2_sj=0; 
			if(FFT_Mag[228]<400)
			f2_sj=1; 
		}break;
	}
	
	if(f1_sj==1)
	{
		f1_sin=0;
	}
	else
	{
		f1_sin=1;
	}
	if(f2_sj==1)
	{
		f2_sin=0;
	}
	else
	{
		f2_sin=1;
	}
	if(cycs==3) //取第三个采样周期来初始化双路继电器
	{
		if(f1_sj==1)
		f1_SJout();//切换继电器到fA三角波输出
		else
		f1_Sinout();//切换继电器到fA正弦波输出
		
		if(f2_sj==1)
		f2_SJout();//切换继电器到fB三角波输出
		else
		f2_Sinout();//切换继电器到fA三角波输出
	}
	
	//显示f1 f2类型 1为真 0为假
	LCD_ShowNum(500,100,f1_sin,1,24); 
	LCD_ShowNum(500,130,f1_sj,1,24);  
	LCD_ShowNum(500,300,f2_sin,1,24);
	LCD_ShowNum(500,330,f2_sj,1,24);

}

/******************************************************************
简介：DMA中断用于完整采样一次（采样1024次），
	  并将其存储于adcx[]缓存数组中，等待后续数据处理
*******************************************************************/	
void DMA1_Channel1_IRQHandler(void) 
{
	if(DMA_GetITStatus(DMA1_IT_TC1)!=RESET)
	{
		adcx[t]=currentadc;
		t++;
		if(ddgdp_flag==1)//减小漂移
		{
			if((adcx[t-1]>=ADCmax[7])||(adcx[t-1]>=ADCmax[8]))
			{
				ht_flag=1;
				ddgdp_flag=2;
        //立即启动dds
				IO_Update();	//AD9959更新数据,调用此函数后，上述操作生效！！！！
			}
		}
		if(ddgdp_flag==2)//减小漂移
		{
			if((adcx[t-1]>=ADCmax[7])||(adcx[t-1]>=ADCmax[8]))
			{
				IO_Update();	//AD9959更新数据,调用此函数后，上述操作生效！！！！
					t=0;
			}
			else
			t=0;
		}
		if(t==NPT)
		{
			t=0;
			adc_flag=1;
			DMA_Cmd(DMA1_Channel1, DISABLE);        //失能DMA
		}
	}
	DMA_ClearITPendingBit(DMA1_IT_TC1);
}

/******************************************************************
简介：定时器3中断服务函数，用于正弦波输出（H题无用）
	  每进入一次中断改变一次DCA输出值
*******************************************************************/
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
	{
		sinout();
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update);  //清除TIMx的中断待处理位:TIM 中断源 
	}
}

/******************************************************************
简介：三个外部中断用于按键的读取
WK_UP按键改变fA fB的相位差 精度为5度 范围（0-180度）
KEY1按键减小采样频率（H题无用）
KEY0按键增加采样频率（H题无用）
*******************************************************************/
void EXTI0_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(WK_UP==1)	 	 //WK_UP按键
	{	
//		show_flag=!show_flag; //启动/停止示波器
//		POINT_COLOR=MAGENTA;
//		if(show_flag)
//			LCD_ShowString(260,128,200,16,16,"ing...");
//		else
//			LCD_ShowString(260,128,200,16,16,"stop");
		Phase+=5;
		if(Phase>180)
		Phase=0;
		AD9959_Set_Phase(CH1, Phase/180.0*8192);
		IO_Update();
		POINT_COLOR=BLUE;
		LCD_ShowNum(570,200,Phase,3,24);//屏幕显示A B波相位差
	}
	EXTI_ClearITPendingBit(EXTI_Line0); //清除LINE0上的中断标志位  
}
 
void EXTI3_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY1==0)	 //按键KEY1
	{	
//		BEEP=1;
//		delay_ms(50);
//		BEEP=0;
//		pre=pre+5;
//		if(pre>72)
//		{
//			pre=1;
//		}
//		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);//PSC立即被加载
	}		 
	EXTI_ClearITPendingBit(EXTI_Line3);  //清除LINE3上的中断标志位  
}

void EXTI4_IRQHandler(void)
{
	delay_ms(10);//消抖
	if(KEY0==0)	 //按键KEY0
	{
//		BEEP=1;
//		delay_ms(50);
//		BEEP=0;
//		pre=pre-5;
//		if(pre<=1)
//		{
//			pre=1;
//		}
//		TIM_PrescalerConfig(TIM2,pre-1,TIM_PSCReloadMode_Immediate);
	}		 
	EXTI_ClearITPendingBit(EXTI_Line4);  //清除LINE4上的中断标志位  
}

/******************************************************************
函数名称:InitBufInArray()（H题无用）
函数功能:正弦波值初始化，将正弦波各点的值存入magout[]数组中
参数说明:
备    注:需要拔掉WIFI模块，否则输出电压出错
*******************************************************************/
void InitBufInArray(void)
{
    u16 i;
    float fx;
    for(i=0; i<NPT; i++)
    {
        fx = sin((PI2*i)/NPT);
        magout[i] = (u16)(2048+2048*fx);
    }
}

/**********************************************************
简介：画点函数，反转Y坐标
***********************************************************/
void lcd_huadian(u16 a,u16 b,u16 color)
{							    
	LCD_Fast_DrawPoint(a,240-b,color);
}

/**********************************************************
简介：画线函数，反转Y坐标
***********************************************************/
void lcd_huaxian(u16 x1,u16 y1,u16 x2,u16 y2)
{
	LCD_DrawLine(x1,240-y1,x2,240-y2);
}

/**********************************************************
简介：主界面绘制
***********************************************************/
void window(void)
{
	u16 x,i;
	static u16 h; 
	
	POINT_COLOR=GREEN;	  
	LCD_ShowString(5,8,200,24,24,"OSC-DWY"); //名称
	
	POINT_COLOR=GRAY;
	LCD_ShowString(190,13,200,16,16,"mV/div");	//纵坐标分度值
	LCD_ShowString(260,5,200,16,16,"max(mv):"); //最大电压
	LCD_ShowString(260,45,200,16,16,"min(mv):");//最小电压
	LCD_ShowString(260,85,200,16,16,"vpp(mv):");//峰峰值
	LCD_ShowString(100,300,200,24,24,"fA(KHz):");//A波频率
	LCD_ShowString(260,300,200,24,24,"fB(KHz):");//B波频率
	LCD_ShowString(180,360,200,24,24,"OSR:(KHz)");  //采样频率	
	
	LCD_ShowString(420,100,200,24,24,"SIN B:"); //A波正弦波标志位 B为波
	LCD_ShowString(420,130,200,24,24,"SJ B:");  //A波三角波标志位
	LCD_ShowString(420,300,200,24,24,"SIN B:"); //B波正弦波标志位
	LCD_ShowString(420,330,200,24,24,"SJ B:");  //B波三角波标志位
	
	POINT_COLOR=BRRED;
	LCD_ShowString(100,13,200,16,16,"IN:PC4"); //ADC模拟输入引脚PC4
	LCD_ShowString(420,20,200,24,24,"fA LX:"); //A波 类型
	LCD_ShowString(420,220,200,24,24,"fB LX:");//B波 类型

	POINT_COLOR=BRRED;
	LCD_ShowString(570,170,200,24,24,"Phase:");//A B波相位差
	
	
	POINT_COLOR=BLUE;
	LCD_ShowNum(150,13,V,4,16);//mv/div
	LCD_ShowNum(570,200,Phase,3,24); //A B波相位差
	
	
	POINT_COLOR=BLUE;
	fre=72000000/T/pre;//更新采样频率(HZ)
	FRE=fre/1000;//更新采样频率(KHZ)
	LCD_ShowNum(180,390,FRE,5,24);//更新采样率显示

	
	POINT_COLOR=WHITE;			
	lcd_huaxian(0,0,0,200);//画四格框
	lcd_huaxian(256,0,256,200);
	lcd_huaxian(0,0,256,0);		
	lcd_huaxian(0,200,256,200);
	
	for(x=0;x<256;x++)
	{
		lcd_huadian(x,100,WHITE);
		if(x == table[h])	
		{
			POINT_COLOR=WHITE;	
			lcd_huaxian(x,1,x,3);
			lcd_huaxian(x,101,x,103);
			lcd_huaxian(x,199,x,197);
			h++;
			if(h>=16) h=0;
		}	
		if(x==128) 
		{
			lcd_huaxian(x,1,x,199);
			for(i=10;i<200;i+=10)
			{
				lcd_huaxian(125,i,127,i);
			}
		}
	}
	
	POINT_COLOR=MAGENTA;	
	LCD_ShowString(260,128,200,16,16,"ing..."); //示波器正在运行
}

/******************************************************************
函数名称:clear_point()（H题前三天有效）可以充当小型示波器看硬件的输出波形是否有问题
能实时更新A B波的频率和类型 第四天为了减小漂移 没有绘图
函数功能:循环更新波形
参数说明:mode 波形模式选择 1——连线模式，0——打点模式
备    注:波形的显示可采用打点方式和绘制线方式
*******************************************************************/
void clear_point(u16 mode)
{
	u16 x,i,past_vol,pre_vol;
	static u16 h; 
	
	
	for(x=0;x<256;x++)
	{	
		POINT_COLOR=BLACK;	//按列清除
		if(x!=128)	//y轴不进行列清除
			lcd_huaxian(x,4,x,196);
		
		//绘制坐标
		POINT_COLOR=WHITE;
		lcd_huaxian(0,0,0,200);
		lcd_huadian(x,100,WHITE);
		if(x == table[h])	
		{
			lcd_huaxian(x,101,x,103);
			h++;
			if(h>=15) h=0;
		}
		if(x==128) 
		{
			lcd_huaxian(x,1,x,199);
			for(i=10;i<200;i+=10)
			{
				lcd_huaxian(125,i,127,i);
			}
		}
		
		pre_vol = adcx[x]/4095.0*100*2;

		//波形更新
		if(mode==1)
		{
			POINT_COLOR=YELLOW;
			if(x>0&&x<255&&x!=128)	//去除第一个，最后一个以及y轴上点的绘制
			lcd_huaxian(x,past_vol,x+1,pre_vol);
		}
		else
			lcd_huadian(x,pre_vol,YELLOW);

		past_vol = pre_vol;
	}
	
}

/******************************************************************
函数名称:sinout()（H题无用）
函数功能:正弦波输出
参数说明:
备    注:将此函数置于定时器中断中，可模拟输出正弦波
*******************************************************************/
void sinout(void)
{
	static u16 i=0;
	DAC_SetChannel1Data(DAC_Align_12b_R,magout[i]);
	i++;
	if(i>=NPT)
		i=0;
}

