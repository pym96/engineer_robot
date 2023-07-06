#ifndef __APP_LED_H
#define __APP_LED_H

//#include "dev_system.h"

class LEDctrl : public LEDdev
{
public:
  void Turn_On();
  void Turn_On( Pin_TypeDef Pin );

  void Turn_Off();
  void Turn_Off( Pin_TypeDef Pin );

  void Toggle();
  void Toggle( Pin_TypeDef Pin );
};

extern LEDctrl LED;

#endif /* __APP_LED_H */
