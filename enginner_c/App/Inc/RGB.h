#ifndef  __RGB_H
#define __RGB_H

#include "stdint.h"

/* WS2812B f=800k, T=1.25us ,Arr=209*/
#define ONE_PULSE           (143)          	//1��(2/3*T) ռ�ձ�=CCRX/(ARR+1)
#define ZERO_PULSE          (67)           	//0��(1/3*T)
#define RESET_PULSE         (9000)         	//�͵�ƽ��λ�ź�50us����(1����Ԫ��ʱ������210�Σ�����1.25us����50us,��Ҫ40����Ԫ����ʱ��������Ҫ����8400��)
#define LED_NUMS            (8) 		   			//��������
#define LED_DATA_LEN        (24)           	//RGB��ɫ���ݳ���, һ��������Ҫ24bits
#define DEFAULT_BRIGHTNESS  (100)          	//�ƴ�Ĭ������:   	100
#define SATURATION          (100)          	//�ư�Ĭ�ϱ��Ͷȣ�	100
#define WS2812_DATA_LEN (RESET_PULSE + LED_NUMS * LED_DATA_LEN) //ws2812������Ҫ�������鳤��
#define LOOP_ALL for(size_t i = 0; i < LED_NUMS; i++) /* ���е��� */

/* HSV��ʽ����ɫֵԤ�� */
#define RED 0
#define YELLOW 60
#define GREEN 120
#define CYAN 180
#define BLUE 240

void WS2812_INIT(void);
void WS2812_SET_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num);
void WS2812_SET_HSV(uint16_t H, uint16_t S, uint16_t V, uint16_t num);
void WS2812_SET_DARK(void);

#endif

