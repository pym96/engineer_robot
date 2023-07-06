/*
 * MIT License
 * Copyright (c) 2019 _VIFEXTech
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */
#include "platform_timer.h"

/*��ʱ�����ö��*/
typedef enum
{
    TIMER1, TIMER2, TIMER3, TIMER4, TIMER5, TIMER6, TIMER7, TIMER8,
    TIMER9, TIMER10, TIMER11, TIMER12, TIMER13, TIMER14, TIMER_MAX
} TIMER_Type;

/*��ʱ�жϻص�����ָ������*/
static Timer_CallbackFunction_t TIMx_Function[TIMER_MAX] = {0};

/**
  * @brief  ������ر�ָ����ʱ����ʱ��
  * @param  TIMx:��ʱ����ַ
  * @param  NewState: ENABLE������DISABLE�ر�
  * @retval ��
  */
void Timer_ClockCmd(TIM_TypeDef* TIMx, FunctionalState NewState)
{
    if(TIMx == TIM1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, NewState);
    }
    else if(TIMx == TIM2)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, NewState);
    }
    else if(TIMx == TIM3)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, NewState);
    }
    else if(TIMx == TIM4)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, NewState);
    }
    else if(TIMx == TIM5)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, NewState);
    }
    else if(TIMx == TIM6)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM6, NewState);
    }
    else if(TIMx == TIM7)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, NewState);
    }
    else if(TIMx == TIM8)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, NewState);
    }
    else if(TIMx == TIM9)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM9, NewState);
    }
    else if(TIMx == TIM10)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM10, NewState);
    }
    else if(TIMx == TIM11)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM11, NewState);
    }
    else if(TIMx == TIM12)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM12, NewState);
    }
    else if(TIMx == TIM13)
    {
        RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM13, NewState);
    }
    else
    {
        if(TIMx == TIM14)
        {
            RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM14, NewState);
        }
    }
}

/*ȡ����ֵ*/
#define CLOCK_ABS(x) (((x)>0)?(x):-(x))

/*������ƽ����*/
static float Qsqrt(float number)
{
    long i;
    float x2, y;
    const float threehalfs = 1.5f;
    x2 = number * 0.5f;
    y  = number;
    i  = *(long*)&y;
    i  = 0x5f3759df - (i >> 1);
    y  = *(float*)&i;
    y  = y * (threehalfs - (x2 * y * y));
    y  = y * (threehalfs - (x2 * y * y));
    return 1.0f / y;
}

/**
  * @brief  ����ʱ�ж�Ƶ��ת��Ϊ��װֵ��ʱ�ӷ�Ƶֵ
  * @param  freq: �ж�Ƶ��(Hz)
  * @param  clock: ��ʱ��ʱ��
  * @param  *period: ��װֵ��ַ
  * @param  *prescaler: ʱ�ӷ�Ƶֵ��ַ
  * @retval ���ֵ(Hz)
  */
static int32_t Timer_FreqToArrPsc(
    uint32_t freq, uint32_t clock,
    uint16_t *period, uint16_t *prescaler
)
{
    uint32_t prodect;
    uint16_t psc, arr;
    uint16_t max_error = 0xFFFF;

    if(freq == 0 || freq > clock)
        goto failed;

    /*��ȡarr��pscĿ��˻�*/
    prodect = clock / freq;

    /*��prodect��ƽ������ʼ����*/
    psc = Qsqrt(prodect);

    /*������ʹarr*psc�㹻�ӽ�prodect*/
    for(; psc > 1; psc--)
    {
        for(arr = psc; arr < 0xFFFF; arr++)
        {
            /*�����*/
            int32_t newerr = arr * psc - prodect;
            newerr = CLOCK_ABS(newerr);
            if(newerr < max_error)
            {
                /*������С���*/
                max_error = newerr;
                /*����arr��psc*/
                *period = arr;
                *prescaler = psc;
                /*���*/
                if(max_error == 0)
                    goto success;
            }
        }
    }

    /*����ɹ�*/
success:
    return (freq - clock / ((*period) * (*prescaler)));

    /*ʧ��*/
failed:
    return (freq - clock);
}

/**
  * @brief  ����ʱ�ж�ʱ��ת��Ϊ��װֵ��ʱ�ӷ�Ƶֵ
  * @param  time: �ж�ʱ��(΢��)
  * @param  clock: ��ʱ��ʱ��
  * @param  *period: ��װֵ��ַ
  * @param  *prescaler: ʱ�ӷ�Ƶֵ��ַ
  * @retval ��
  */
static void Timer_TimeToArrPsc(
    uint32_t time, uint32_t clock,
    uint16_t *period, uint16_t *prescaler
)
{
    uint32_t cyclesPerMicros = clock / 1000000U;
    uint32_t prodect = time * cyclesPerMicros;
    uint16_t arr, psc;

    if(prodect < cyclesPerMicros * 30)
    {
        arr = 10;
        psc = prodect / arr;
    }
    else if(prodect < 65535 * 1000)
    {
        arr = prodect / 1000;
        psc = prodect / arr;
    }
    else
    {
        arr = prodect / 20000;
        psc = prodect / arr;
    }
    *period = arr;
    *prescaler = psc;
}

/**
  * @brief  ��ʱ�ж�����
  * @param  TIMx:��ʱ����ַ
  * @param  time: �ж�ʱ��(΢��)
  * @param  function: ��ʱ�жϻص�����
  * @retval ��
  */
void Timer_SetInterrupt(TIM_TypeDef* TIMx, uint32_t time, Timer_CallbackFunction_t function)
{
    uint16_t period, prescaler;
    uint32_t clock = (IS_APB2_TIM(TIMx) ? F_CPU : (F_CPU / 2));

    if(!IS_TIM_ALL_PERIPH(TIMx) || time == 0)
        return;

    /*����ʱ�ж�ʱ��ת��Ϊ��װֵ��ʱ�ӷ�Ƶֵ*/
    Timer_TimeToArrPsc(
        time,
        clock,
        &period,
        &prescaler
    );

    /*��ʱ�ж�����*/
    Timer_SetInterruptBase(
        TIMx,
        period,
        prescaler,
        function,
        Timer_PreemptionPriority_Default,
        Timer_SubPriority_Default
    );
}

/**
  * @brief  ���¶�ʱ�ж�Ƶ��
  * @param  TIMx:��ʱ����ַ
  * @param  freq:�ж�Ƶ��
  * @retval ��
  */
void Timer_SetInterruptFreqUpdate(TIM_TypeDef* TIMx, uint32_t freq)
{
    uint16_t period, prescaler;
    uint32_t clock = (IS_APB2_TIM(TIMx) ? F_CPU : (F_CPU / 2));

    if(!IS_TIM_ALL_PERIPH(TIMx) || freq == 0)
        return;

    Timer_FreqToArrPsc(
        freq,
        clock,
        &period,
        &prescaler
    );
    TIM_SetAutoreload(TIMx, period - 1);
    TIM_PrescalerConfig(TIMx, prescaler - 1, TIM_PSCReloadMode_Immediate);
}

/**
  * @brief  ��ȡ��ʱ���ж�Ƶ��
  * @param  TIMx:��ʱ����ַ
  * @retval �ж�Ƶ��
  */
uint32_t Timer_GetClockOut(TIM_TypeDef* TIMx)
{
    uint32_t clock = (IS_APB2_TIM(TIMx) ? F_CPU : (F_CPU / 2));
    if(!IS_TIM_ALL_PERIPH(TIMx))
        return 0;

    return (clock / ((TIMx->ARR + 1) * (TIMx->PSC + 1)));
}

/**
  * @brief  ���¶�ʱ�ж�ʱ��
  * @param  TIMx:��ʱ����ַ
  * @param  time: �ж�ʱ��(΢��)
  * @retval ��
  */
void Timer_SetInterruptTimeUpdate(TIM_TypeDef* TIMx, uint32_t time)
{
    uint16_t period, prescaler;
    uint32_t clock = (IS_APB2_TIM(TIMx) ? F_CPU : (F_CPU / 2));

    if(!IS_TIM_ALL_PERIPH(TIMx))
        return;

    Timer_TimeToArrPsc(
        time,
        clock,
        &period,
        &prescaler
    );

    TIM_SetAutoreload(TIMx, period - 1);
    TIM_PrescalerConfig(TIMx, prescaler - 1, TIM_PSCReloadMode_Immediate);
}

/**
  * @brief  ��ʱ�жϻ�������
  * @param  TIMx:��ʱ����ַ
  * @param  period:��װֵ
  * @param  prescaler:ʱ�ӷ�Ƶֵ
  * @param  function: ��ʱ�жϻص�����
  * @param  PreemptionPriority: ��ռ���ȼ�
  * @param  SubPriority: �����ȼ�
  * @retval ��
  */
void Timer_SetInterruptBase(
    TIM_TypeDef* TIMx,
    uint16_t period, uint16_t prescaler,
    Timer_CallbackFunction_t function,
    uint8_t PreemptionPriority, uint8_t SubPriority
)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
    NVIC_InitTypeDef NVIC_InitStructure;
    uint8_t TIMx_IRQn;
    TIMER_Type TIMERx;

    if(!IS_TIM_ALL_PERIPH(TIMx))
        return;

#define TIMx_IRQn_DEF(n,x_IRQn)\
do{\
    if(TIMx == TIM##n)\
    {\
        TIMERx = TIMER##n;\
        TIMx_IRQn = x_IRQn;\
    }\
}\
while(0)

    /*�����ʾ 
     *identifier "xxx_IRQn" is undefined
     *��δ�����ע�͵�����
     */
    TIMx_IRQn_DEF(1, TIM1_UP_TIM10_IRQn);
    TIMx_IRQn_DEF(2, TIM2_IRQn);
    TIMx_IRQn_DEF(3, TIM3_IRQn);
    TIMx_IRQn_DEF(4, TIM4_IRQn);
    TIMx_IRQn_DEF(5, TIM5_IRQn);
    TIMx_IRQn_DEF(6, TIM6_DAC_IRQn);
    TIMx_IRQn_DEF(7, TIM7_IRQn);
    TIMx_IRQn_DEF(8, TIM8_UP_TIM13_IRQn);
    TIMx_IRQn_DEF(9, TIM1_BRK_TIM9_IRQn);
    TIMx_IRQn_DEF(10, TIM1_UP_TIM10_IRQn);
    TIMx_IRQn_DEF(11, TIM1_TRG_COM_TIM11_IRQn);
    TIMx_IRQn_DEF(12, TIM8_BRK_TIM12_IRQn);
    TIMx_IRQn_DEF(13, TIM8_UP_TIM13_IRQn);
    TIMx_IRQn_DEF(14, TIM8_TRG_COM_TIM14_IRQn);

    if(TIMx_IRQn == 0)
        return;

    /*register callback function*/
    TIMx_Function[TIMERx] = function;

    /*Enable PeriphClock*/
    TIM_DeInit(TIMx);
    Timer_ClockCmd(TIMx, ENABLE);

    TIM_TimeBaseStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseStructure.TIM_Period = period - 1;         //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ
    TIM_TimeBaseStructure.TIM_Prescaler = prescaler - 1;  //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  10Khz�ļ���Ƶ��
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //����ʱ�ӷָ�:TDTS = Tck_tim
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
    TIM_TimeBaseInit(TIMx, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

    /**********************************�����ж����ȼ�************************************/
    NVIC_InitStructure.NVIC_IRQChannel = TIMx_IRQn;  //TIM�ж�
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = PreemptionPriority;  //��ռ���ȼ�
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = SubPriority;  //�����ȼ�
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
    NVIC_Init(&NVIC_InitStructure);  //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ���

    TIM_ClearFlag(TIMx, TIM_FLAG_Update);
    TIM_ITConfig(TIMx, TIM_IT_Update, ENABLE);  //ʹ��TIM�ж�
}

#define TIMx_IRQHANDLER(n) \
do{\
    if (TIM_GetITStatus(TIM##n, TIM_IT_Update) != RESET)\
    {\
        if(TIMx_Function[TIMER##n]) TIMx_Function[TIMER##n]();\
        TIM_ClearITPendingBit(TIM##n, TIM_IT_Update);\
    }\
}while(0)

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��1
  * @param  ��
  * @retval ��
  */
void TIM1_UP_TIM10_IRQHandler(void)   //TIM1�ж�
{
    TIMx_IRQHANDLER(1);
    TIMx_IRQHANDLER(10);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��2
  * @param  ��
  * @retval ��
  */
void TIM2_IRQHandler(void)
{
    TIMx_IRQHANDLER(2);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��3
  * @param  ��
  * @retval ��
  */
void TIM3_IRQHandler(void)
{
    TIMx_IRQHANDLER(3);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��4
  * @param  ��
  * @retval ��
  */
void TIM4_IRQHandler(void)
{
    TIMx_IRQHANDLER(4);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��5
  * @param  ��
  * @retval ��
  */
void TIM5_IRQHandler(void)
{
    TIMx_IRQHANDLER(5);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��6
  * @param  ��
  * @retval ��
  */
void TIM6_DAC_IRQHandler(void)
{
    TIMx_IRQHANDLER(6);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��7
  * @param  ��
  * @retval ��
  */
void TIM7_IRQHandler(void)
{
    TIMx_IRQHANDLER(7);
}

/**
  * @brief  ��ʱ�ж���ڣ���ʱ��8
  * @param  ��
  * @retval ��
  */
void TIM8_UP_TIM13_IRQHandler(void)
{
    TIMx_IRQHANDLER(8);
    TIMx_IRQHANDLER(13);
}
