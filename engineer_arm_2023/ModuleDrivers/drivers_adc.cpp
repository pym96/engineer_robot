#include "drivers_adc.h"

ADCdev::ADCdev( ADC_TypeDef *ADCx, uint8_t ADC_Channelx )
{
    this->ADCx = ADCx;
	  this->ADC_Channelx = ADC_Channelx;
}

void ADCdev::ADCx_Init( Pin_Type ADCx_Pin )
{
    ADC_InitTypeDef ADC_InitStructure;
	
	  GPIOx_Init( ADCx_Pin, INPUT_ANALOG, HIGH_SPEED );
 
    if(ADCx == ADC1)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    }
    else if(ADCx == ADC2)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC2, ENABLE);
    }
    else if(ADCx == ADC3)
    {
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC3, ENABLE);
    }
    else
    {
        return;
    }
    
    RCC_ADCCLKConfig( RCC_PCLK2_Div6 );

    ADC_DeInit(ADCx);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADCx, &ADC_InitStructure);

    ADC_Cmd(ADCx, ENABLE);
    ADC_ResetCalibration(ADCx);
    while(ADC_GetResetCalibrationStatus(ADCx));
    ADC_StartCalibration(ADCx);
    while(ADC_GetCalibrationStatus(ADCx));
}
