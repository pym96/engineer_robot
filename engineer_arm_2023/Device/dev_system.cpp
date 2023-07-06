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
#include "dev_system.h"

/**
  * @brief  ���������������ģʽ
  * @param  Pin: ���ű��
  * @param  pinMode_x: ģʽ
  * @retval ��
  */
void pinMode(uint8_t Pin, pinMode_TypeDef pinMode_x)
{
    if(!IS_PIN(Pin))
        return;
    
    if(pinMode_x == INPUT_ANALOG_DMA)
    {
        if(!IS_ADC_PIN(Pin))
            return;
        
        pinMode(Pin, INPUT_ANALOG);
    }
    else if(pinMode_x == PWM)
    {
        PWM_Init(Pin, 1000, 10000);
    }
    else
    {
        GPIOx_Init(
            Pin, 
            pinMode_x, 
            GPIO_Speed_100MHz
        );
    }
}

/**
  * @brief  ������HIGH(1)��LOW(0)ֵд����������
  * @param  Pin:���ű��
  * @param  val: д��ֵ
  * @retval ��
  */
void digitalWrite(uint8_t Pin, uint8_t val)
{
    if(!IS_PIN(Pin))
        return;
    
    val ? digitalWrite_HIGH(Pin) : digitalWrite_LOW(Pin);
}

/**
  * @brief  ��ȡ���ŵ�ƽ
  * @param  Pin: ���ű��
  * @retval ���ŵ�ƽ
  */
uint8_t digitalRead(uint8_t Pin)
{
    if(!IS_PIN(Pin))
        return 0;
    
    return digitalRead_FAST(Pin);
}

/**
  * @brief  ��ģ��ֵ(PWMռ�ձ�)д������
  * @param  Pin: ���ű��
  * @param  val: PWMռ�ձ�
  * @retval PWMռ�ձ�
  */
uint16_t analogWrite(uint8_t Pin, uint16_t val)
{
    if(!IS_PIN(Pin))
        return 0;
    
    return (IS_PWM_PIN(Pin) ? pwmWrite(Pin, val) : 0);
}

/**
  * @brief  һ���Ƴ�һ���ֽڵ����ݣ�������˻���С(���ұ�)��ʼ
  * @param  dataPin: ���ÿ��λ�� pin
  * @param  clockPin: �� dataPin ����Ϊ��ȷֵ��Ҫ�л��� pin (int)
  * @param  bitOrder: MSBFIRST / LSBFIRST
  * @param  value: Ҫ�Ƴ�������(�ֽ�)
  * @retval ��
  */
void shiftOut(uint8_t dataPin, uint8_t clockPin, uint8_t bitOrder, uint8_t value)
{
    uint8_t i;
    if(!(IS_PIN(dataPin) && IS_PIN(clockPin)))
        return;
    
    digitalWrite_LOW(clockPin);
    for (i = 0; i < 8; i++)
    {
        int bit = bitOrder == LSBFIRST ? i : (7 - i);
        digitalWrite(dataPin, (value >> bit) & 0x1);
        togglePin(clockPin);
        togglePin(clockPin);
    }
}

/**
  * @brief  һ�ν�һ���ֽڵ�������λ��������˻���С (���ұ�) ��ʼ
  * @param  dataPin:  ����ÿ��λ�� pin
  * @param  clockPin: Ҫ�л����� pin �źŴ�dataPin��ȡ
  * @param  bitOrder: MSBFIRST/LSBFIRST
  * @retval ��ȡ��ֵ (�ֽ�)
  */
uint32_t shiftIn(uint8_t dataPin, uint8_t clockPin, uint32_t bitOrder)
{
    uint8_t value = 0 ;
    uint8_t i ;
    
    if(!(IS_PIN(dataPin) && IS_PIN(clockPin)))
        return 0;
    
    for ( i = 0 ; i < 8 ; ++i )
    {
        digitalWrite_HIGH(clockPin) ;
        if (bitOrder == LSBFIRST )
        {
            value |= digitalRead(dataPin) << i ;
        }
        else
        {
            value |= digitalRead(dataPin) << (7 - i) ;
        }
        digitalWrite_LOW(clockPin);
    }

    return value ;
}

/**
  * @brief  ��һ�����ִ�һ����Χ����ӳ�䵽��һ������
  * @param  x: Ҫӳ�������
  * @param  in_min: ֵ�ĵ�ǰ��Χ���½�
  * @param  in_max: ֵ�ĵ�ǰ��Χ���Ͻ�
  * @param  out_min: ֵ��Ŀ�귶Χ���½�
  * @param  out_max: ֵĿ�귶Χ���Ͻ�
  * @retval ӳ���ֵ
  */
template<typename T>
T fmap(T x, T in_min, T in_max, T out_min, T out_max)
{
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
