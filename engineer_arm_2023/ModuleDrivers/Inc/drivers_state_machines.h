#ifndef __DRIVERS_STATE_MACHINES_H
#define __DRIVERS_STATE_MACHINES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdlib.h>
#include <stdio.h>
	
#ifdef __cplusplus
}
#endif

typedef enum WorkMode
{
	NO_MOVE=0,
	FOLLOW_YAW,//¸úËæÔÆÌ¨
	LITTLE_TOP,//Ð¡ÍÓÂÝ
	VISION,
}WorkMode_e;

typedef struct
{
	WorkMode work_mode;
	bool (*modecheck)(void);
	WorkMode next_work_mode;
}ModeCheck_t;

typedef struct
{
	WorkMode work_mode;
	void (*modeInit)(void);
	void (*modeLoop)(void);
}ModePerform_t;

class State_Switch{
public:
	State_Switch( unsigned int Number )
	{
	  State_Number = Number;
		
		this->ModeCheck =   ( ModeCheck_t *)malloc( Number );
		this->ModePerform = ( ModePerform_t *)malloc( Number );
		
		Mode_Now = NO_MOVE;
		Mode_Last = FOLLOW_YAW;
	}
	
	ModeCheck_t   *ModeCheck;
  ModePerform_t *ModePerform;

  void ModeCheckPerform( void );
  void WorkAllPerform( void );
private:
	unsigned int State_Number;

	WorkMode Mode_Now;
  WorkMode Mode_Last;
};

#endif /* __DRIVERS_STATE_MACHINES_H */
