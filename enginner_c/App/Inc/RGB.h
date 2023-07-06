#ifndef  __RGB_H
#define __RGB_H

#include "stdint.h"

/* WS2812B f=800k, T=1.25us ,Arr=209*/
#define ONE_PULSE           (143)          	//1码(2/3*T) 占空比=CCRX/(ARR+1)
#define ZERO_PULSE          (67)           	//0码(1/3*T)
#define RESET_PULSE         (9000)         	//低电平复位信号50us以上(1个码元定时器计数210次，花费1.25us；则50us,需要40个码元，定时器最少需要计数8400次)
#define LED_NUMS            (8) 		   			//灯珠数量
#define LED_DATA_LEN        (24)           	//RGB颜色数据长度, 一个灯珠需要24bits
#define DEFAULT_BRIGHTNESS  (100)          	//灯带默认亮度:   	100
#define SATURATION          (100)          	//灯板默认饱和度：	100
#define WS2812_DATA_LEN (RESET_PULSE + LED_NUMS * LED_DATA_LEN) //ws2812灯条需要的总数组长度
#define LOOP_ALL for(size_t i = 0; i < LED_NUMS; i++) /* 所有灯珠 */

/* HSV格式常用色值预设 */
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

