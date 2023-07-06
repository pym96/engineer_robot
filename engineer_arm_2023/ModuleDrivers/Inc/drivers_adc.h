#ifndef __DRIVERS_ADC_H
#define __DRIVERS_ADC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#ifdef __cplusplus
}
#endif

class ADCdev
{
public:
	ADCdev( ADC_TypeDef *ADCx, uint8_t ADC_Channelx );

  void ADCx_Init( Pin_Type ADCx_Pin );
protected:
	ADC_TypeDef *ADCx;
  uint8_t ADC_Channelx;
  uint32_t ADC_ConvertedValue;
};

#endif /* __DRIVERS_ADC_H */
