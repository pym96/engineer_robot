#include "drivers_state_machines.h"

void State_Switch::ModeCheckPerform( void )
{
  for( int i = 0; i < State_Number; i++ )
	{
		if( Mode_Now == ModeCheck[i].work_mode )
		{	
			if( ModeCheck[i].modecheck() == true )
			{
				Mode_Now = ModeCheck[i].next_work_mode;
			}
		}
	}
}
	
void State_Switch::WorkAllPerform( void )
{
	for( int i = 0; i < State_Number; i++ )
	{
		if( Mode_Now == ModePerform[i].work_mode )
		{
			if( Mode_Last != Mode_Now )
			{
				ModePerform[i].modeInit();
				Mode_Last = Mode_Now;
			}
			ModePerform[i].modeLoop();
			ModeCheckPerform();
		}
	}
}
