#ifndef INC_INCLUDE_TCA9535_H_
#define INC_INCLUDE_TCA9535_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "debug.h"


#define TCA9535_I2C_ADDRESS	0x40

typedef enum
{
	Input_Port_0_Reg,
	Input_Port_1_Reg,
	Output_Port_0_Reg,
	Output_Port_1_Reg,
	Polarity_Inversion_Port_0,
	Polarity_Inversion_Port_1,
	Configuration_Port_0,
	Configuration_Port_1
} TCA9535_register;

void TCA9535_Init();
HAL_StatusTypeDef TCA9535_Set_Register(TCA9535_register reg, uint8_t value);
HAL_StatusTypeDef TCA9535_Get_Register(TCA9535_register reg, uint8_t* data);
HAL_StatusTypeDef TCA9535_drive_LEDs(uint8_t value);
uint8_t TCA9535_get_LEDs();

#endif /* INC_INCLUDE_TCA9535_H_ */
