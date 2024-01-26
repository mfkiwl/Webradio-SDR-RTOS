#include <FT5426.h>


void FT5426_Get_Touch(FT5426_register reg, uint16_t* X, uint16_t* Y, Event_flag *Flag)
{
	HAL_StatusTypeDef Status;
	uint8_t reg_8 = (uint8_t)reg;
	uint8_t data[4];

	Status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)FT5426_I2C_ADDRESS, &reg_8, 1, HAL_MAX_DELAY);
	Status |= HAL_I2C_Master_Receive(&hi2c1, (uint16_t)FT5426_I2C_ADDRESS, data, 4, HAL_MAX_DELAY);

	*X = (uint16_t) (((0x0F & data[0]) << 8) & data[1]);
	*Y = (uint16_t) (((0x0F & data[2]) << 8) & data[3]);
	*Flag = (Event_flag) ((0xF0 & data[0]) >> 4);

	if(Status != HAL_OK)
		DEBUGOUT("FT5426_Get_Touch error...\r\n");
}

HAL_StatusTypeDef FT5426_Set_Register(FT5426_register reg, uint8_t value)
{
	uint8_t data[2];
	HAL_StatusTypeDef Status;

	data[0] = (uint8_t)reg;
	data[1] = value;
	Status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)FT5426_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);

	if(Status != HAL_OK)
		DEBUGOUT("FT5426_Set_Register error...\r\n");

	return (Status);
}

HAL_StatusTypeDef FT5426_Get_Register(FT5426_register reg, uint8_t* data)
{
	HAL_StatusTypeDef Status;
	uint8_t reg_8 = (uint8_t)reg;

	Status = HAL_I2C_Master_Transmit(&hi2c1, (uint16_t)FT5426_I2C_ADDRESS, &reg_8, 1, HAL_MAX_DELAY);
	Status |= HAL_I2C_Master_Receive(&hi2c1, (uint16_t)FT5426_I2C_ADDRESS, data, 1, HAL_MAX_DELAY);

	if(Status != HAL_OK)
		DEBUGOUT("FT5426_Get_Register error...\r\n");

	return (Status);
}


