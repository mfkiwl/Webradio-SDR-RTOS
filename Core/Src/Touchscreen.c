#include <Touchscreen.h>

void Touchscreen_Init()
{
	HAL_StatusTypeDef Status;

	Status = FT5426_Set_Register(Mode_Switch, WORKING_MODE);

	if(Status != HAL_OK)
			DEBUGOUT("Touchscreen_Init error...\r\n");

	DEBUGOUT("Touchscreen init...\r\n");
}



void Touchscreen_Interrupt()
{

}
