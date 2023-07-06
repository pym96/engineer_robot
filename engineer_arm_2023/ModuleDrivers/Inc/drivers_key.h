#ifndef __DRIVERS_KEY_H
#define __DRIVERS_KEY_H

#ifdef __cplusplus
extern "C" {
#endif

#include "platform.h"
	
#ifdef __cplusplus
}
#endif

class Keydev{
public:
	Keydev( Pin_Type Key_Pin );

	void Key_Init( void );
protected:
  Pin_Type Key_Pin;
};

extern Keydev Key;

#endif /* __DRIVERS_KEY_H */
