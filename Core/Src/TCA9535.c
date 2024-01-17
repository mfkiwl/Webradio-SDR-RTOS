#include <TCA9535.h>

uint8_t TCA9535_LEDs;

void TCA9535_Init()
{
	HAL_StatusTypeDef Status;

	// Set port 0 as 6bit inputs, 1bit output for TFT_TOUCH_RESET and 1bit output for ESP_RESETN
	Status = TCA9535_Set_Register(Configuration_Port_0, 0xFC);
	Status |= TCA9535_Set_Register(Polarity_Inversion_Port_0, 0xFC);
	// Set port 1 as 8bit outputs for LEDS
	Status |= TCA9535_Set_Register(Configuration_Port_1, 0x00);

	if(Status != HAL_OK)
		DEBUGOUT("TCA9535_Init error...\r\n");

	// Light up LED on port P1.0
	TCA9535_drive_LEDs(0x01);
	DEBUGOUT("TCA9535 init...\r\n");
}

HAL_StatusTypeDef TCA9535_Set_Register(TCA9535_register reg, uint8_t value)
{
	uint8_t data[2];
	HAL_StatusTypeDef Status;

	data[0] = (uint8_t)reg;
	data[1] = value;
	Status = HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)TCA9535_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);

	if(Status != HAL_OK)
		DEBUGOUT("TCA9535_Set_Register error...\r\n");

	return (Status);
}

HAL_StatusTypeDef TCA9535_Get_Register(TCA9535_register reg, uint8_t* data)
{
	HAL_StatusTypeDef Status;
	uint8_t reg_8 = (uint8_t)reg;

	Status = HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)TCA9535_I2C_ADDRESS, &reg_8, 1, HAL_MAX_DELAY);
	Status |= HAL_I2C_Master_Receive(&hi2c4, (uint16_t)TCA9535_I2C_ADDRESS, data, 1, HAL_MAX_DELAY);

	if(Status != HAL_OK)
		DEBUGOUT("TCA9535_Get_Register error...\r\n");

	return (Status);
}

HAL_StatusTypeDef TCA9535_drive_LEDs(uint8_t value)
{
	uint8_t data[2];
	HAL_StatusTypeDef Status;

	TCA9535_LEDs = value;

	data[0] = (uint8_t)0x03;
	data[1] = ~value;
	Status = HAL_I2C_Master_Transmit(&hi2c4, (uint16_t)TCA9535_I2C_ADDRESS, data, 2, HAL_MAX_DELAY);

	if(Status != HAL_OK)
		DEBUGOUT("TCA9535_drive_LEDs error...\r\n");

	return (Status);
}

uint8_t TCA9535_get_LEDs()
{
	return TCA9535_LEDs;
}
