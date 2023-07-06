#include "platform_systick.h"

static volatile uint32_t sysTickPerUs  = 168;   /* ϵͳʱ��Ƶ�� - 168MHz */
volatile uint32_t SysTickMillis = 0;
 
/* ����Ƿ���� */
uint32_t Systick_Check_Underflow(void)
{
   return SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk;
}

#ifndef useFreeRTOS

/* systick �жϷ������ */ 
void SysTick_Handler(void)
{
   __disable_irq();
 
   Systick_Check_Underflow();
   
   SysTickMillis++;
 
   __enable_irq();
}
 
void Systick_Init( uint32_t sysTickPerTime )
{
	/* SystemFrequency / 1000    1ms �ж�һ��
   * SystemFrequency / 100000 10us �ж�һ��
   * SystemFrequency / 1000000 1us �ж�һ��
   */
   sysTickPerUs = CYCLES_PER_MICROSECOND;
   
   if(SysTick_Config(sysTickPerTime)){
		 while(1);
	 }
}
#else
static uint8_t fac_us = 0;
static uint32_t fac_ms = 0;
static uint32_t sysTickCnt=0;

extern void xPortSysTickHandler(void);
void SysTick_Handler(void)
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED)
    {
      xPortSysTickHandler();
    }else
		{
		  sysTickCnt++;	/*���ȿ���֮ǰ����*/
		}
}

void delay_init(uint32_t TICK_RATE_HZ)
{
    uint32_t reload = 0;
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8);//180Mhz�˷�Ƶ
    fac_us = SystemCoreClock / 8000000;
    fac_ms = SystemCoreClock / 8000;

    if (TICK_RATE_HZ == 0)
    {
        TICK_RATE_HZ = 1000;
    }

    reload = SystemCoreClock / TICK_RATE_HZ / 8;
    reload--;

    SysTick->CTRL |= SysTick_CTRL_TICKINT_Msk;
    SysTick->LOAD = reload;
    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk;
}

void delay_xus( uint16_t nus )
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nus * fac_us;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}

void delay_xms(uint16_t nms)
{
    uint32_t ticks = 0;
    uint32_t told = 0;
    uint32_t tnow = 0;
    uint32_t tcnt = 0;
    uint32_t reload = 0;
    reload = SysTick->LOAD;
    ticks = nms * fac_ms;
    told = SysTick->VAL;
    while (1)
    {
        tnow = SysTick->VAL;
        if (tnow != told)
        {
            if (tnow < told)
            {
                tcnt += told - tnow;
            }
            else
            {
                tcnt += reload - tnow + told;
            }
            told = tnow;
            if (tcnt >= ticks)
            {
                break;
            }
        }
    }
}
#endif
 
/* ��ȡϵͳ΢�뼶����ʱ�� */ 
uint32_t micros(void)
{
   uint32_t volatile cycle, timeMs;
 
   do
    {
       timeMs = SysTickMillis;
       cycle  = SysTick->VAL;
       
       __ASM volatile("nop");
       
   }while (timeMs != SysTickMillis);
 
   if (Systick_Check_Underflow())
   {
       timeMs++;
       
       cycle = SysTick->VAL;
   }
 
   return (timeMs * 1000) + (SysTick->LOAD + 1 - cycle) / sysTickPerUs;
}
 
/* ��ȡϵͳ���뼶����ʱ�� */ 
uint32_t millis(void)
{
   return SysTickMillis;
}
 
 /* ΢����ʱ���� */
void delay_us(uint32_t us) 
{
    static int currentTime, lastTime = 0;
 
    lastTime= micros();
        
    while(1)
    {       
         currentTime= micros();
 
         if ( currentTime - lastTime >=  us )
         {
            break;
         }
    }
}
 
/* ������ʱ���� */
void delay_ms(uint32_t ms)  
{
    static int currentTime, lastTime = 0;
 
    lastTime= millis();
        
    while(1)
    {       
        currentTime= millis();
 
        if( currentTime - lastTime >=  ms )
        {
            break;
        }
    }
}
			 

