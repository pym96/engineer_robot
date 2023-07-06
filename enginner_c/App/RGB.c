#include "RGB.h"
#include "tim.h"

static uint16_t RGB_buffer[WS2812_DATA_LEN] = {0};
static void WS2812_REFRESH(void);
static void color_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b);
/**
* @brief WS2812初始化, 全黑
*/
void WS2812_INIT(void)
{
    WS2812_SET_DARK();
}

/**
* @brief 	设置某个灯珠颜色RGB
* @author	MA_LU
* @param 	uint8_t R：0-255
*                G：0-255
*                B：0-255
*        	uint16_t num, 指定设置颜色的灯珠位号
*/
void WS2812_SET_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    if (num > LED_NUMS)
        return;

    uint16_t *p = (RGB_buffer + RESET_PULSE) + (num * LED_DATA_LEN);

    for (uint16_t i = 0; i < 8; i++)
    {
        p[i]      = (G << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
    }
		WS2812_REFRESH();
}
/**
* @brief 	设置某个灯珠颜色HSV
* @author MA_LU
* @param 	uint8_t H,S,V: HSV色彩格式
*				 	H:色相(0-360)0度为红色，120度为绿色，240度为蓝色
*				 	S:饱和度(0-100)
*				 	V:亮度(0-100)
*        	uint16_t num, 指定设置颜色的灯珠位号
*/
void WS2812_SET_HSV(uint16_t H, uint16_t S, uint16_t V, uint16_t num)
{
    uint32_t R = 0, G = 0, B = 0;

    if (num > LED_NUMS)
        return;

    uint16_t *p = (RGB_buffer + RESET_PULSE) + (num * LED_DATA_LEN);
		
    color_hsv2rgb(H, S, V, &R, &G, &B);
		
    for (uint16_t i = 0; i < 8; i++)
    {
        p[i]      = (G << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80) ? ONE_PULSE : ZERO_PULSE;
    }
		WS2812_REFRESH();
}

/**
* @brief 灭灯
*/
void WS2812_SET_DARK(void)
{
    LOOP_ALL
    {
        WS2812_SET_RGB(0x00, 0x00, 0x00, i);
    }
}


/**
* @brief WS2812颜色数据刷新, 修改颜色值后调用(DMA传输数据)
*/
static void WS2812_REFRESH(void)
{
    HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_2, (uint32_t *)RGB_buffer, WS2812_DATA_LEN);
}

/**
 * @brief 将HSV颜色空间转换为RGB颜色空间
 * @author INTERNET
 * @param  h HSV颜色空间的H：色调, 范围0~360
 * @param  s HSV颜色空间的S：饱和度, 范围0~100
 * @param  v HSV颜色空间的V：明度, 范围0~100
 * @param  r 转换后RGB-R值的指针
 * @param  g 转换后RGB-G值的指针
 * @param  b 转换后RGB-B值的指针
 *
 */
static void color_hsv2rgb(uint32_t h, uint32_t s, uint32_t v, uint32_t *r, uint32_t *g, uint32_t *b)
{
    h %= 360; // h -> [0,360]
    uint32_t rgb_max = v * 2.55f;
    uint32_t rgb_min = rgb_max * (100 - s) / 100.0f;

    uint32_t i = h / 60;
    uint32_t diff = h % 60;

    uint32_t rgb_adj = (rgb_max - rgb_min) * diff / 60;

    switch (i)
    {
    case 0:
        *r = rgb_max;
        *g = rgb_min + rgb_adj;
        *b = rgb_min;
        break;
    case 1:
        *r = rgb_max - rgb_adj;
        *g = rgb_max;
        *b = rgb_min;
        break;
    case 2:
        *r = rgb_min;
        *g = rgb_max;
        *b = rgb_min + rgb_adj;
        break;
    case 3:
        *r = rgb_min;
        *g = rgb_max - rgb_adj;
        *b = rgb_max;
        break;
    case 4:
        *r = rgb_min + rgb_adj;
        *g = rgb_min;
        *b = rgb_max;
        break;
    default:
        *r = rgb_max;
        *g = rgb_min;
        *b = rgb_max - rgb_adj;
        break;
    }
}

