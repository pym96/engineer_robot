#include "app_power_ctrl.h"

void power_ctrl_configuration(void)
{
	  pinMode( PH2, OUTPUT );
	  pinMode( PH3, OUTPUT );
	  pinMode( PH4, OUTPUT );
	  pinMode( PH5, OUTPUT );

    for (uint8_t i = POWER1_CTRL_SWITCH; i < POWER4_CTRL_SWITCH + 1; i++)
    {
        power_ctrl_off(i);
    }
}

void power_ctrl_on(uint8_t num)
{
    if ( num > POWER4_CTRL_SWITCH )
    {
        return;
    }
    GPIO_SetBits(GPIOH, GPIO_Pin_2 << num);
}

void power_ctrl_off(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_ResetBits(GPIOH, GPIO_Pin_2 << num);
}

void power_ctrl_toggle(uint8_t num)
{
    if (num > POWER4_CTRL_SWITCH)
    {
        return;
    }
    GPIO_ToggleBits(GPIOH, GPIO_Pin_2 << num);
}
