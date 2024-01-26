#ifndef INC_FT5426_H_
#define INC_FT5426_H_

#include "stm32f7xx_hal.h"
#include "main.h"
#include "debug.h"

#define FT5426_I2C_ADDRESS	0x38U

typedef enum
{
	Mode_Switch,
	Gesture,
	Cur_Point,

	Touch1_XH,
	Touch1_XL,
	Touch1_YH,
	Touch1_YL,
	Touch1_Weight,
	Touch1_Misc,

	Touch2_XH,
	Touch2_XL,
	Touch2_YH,
	Touch2_YL,
	Touch2_Weight,
	Touch2_Misc,

	Touch3_XH,
	Touch3_XL,
	Touch3_YH,
	Touch3_YL,
	Touch3_Weight,
	Touch3_Misc,

	Touch4_XH,
	Touch4_XL,
	Touch4_YH,
	Touch4_YL,
	Touch4_Weight,
	Touch4_Misc,

	Touch5_XH,
	Touch5_XL,
	Touch5_YH,
	Touch5_YL,
	Touch5_Weight,
	Touch5_Misc,

	Touch6_XH,
	Touch6_XL,
	Touch6_YH,
	Touch6_YL,
	Touch6_Weight,
	Touch6_Misc,

	Touch7_XH,
	Touch7_XL,
	Touch7_YH,
	Touch7_YL,
	Touch7_Weight,
	Touch7_Misc,

	Touch8_XH,
	Touch8_XL,
	Touch8_YH,
	Touch8_YL,
	Touch8_Weight,
	Touch8_Misc,

	Touch9_XH,
	Touch9_XL,
	Touch9_YH,
	Touch9_YL,
	Touch9_Weight,
	Touch9_Misc,

	Touch10_XH,
	Touch10_XL,
	Touch10_YH,
	Touch10_YL,
	Touch10_Weight,
	Touch10_Misc,
} FT5426_register;

typedef enum
{
	NO_GESTURE	= 0x00,
	MOVE_UP		= 0x10,
	MOVE_RIGHT	= 0x14,
	MOVE_DOWN	= 0x18,
	MOVE_LEFT	= 0x1C,
	ZOOM_IN		= 0x48,
	ZOOM_OUT	= 0x49
} Gesture_ID;

typedef enum
{
	Press_Down,
	Lift_UP,
	Contact,
	No_Event
} Event_flag;

// Device mode
#define WORKING_MODE	0x00
#define TEST_MODE		0x40

void FT5426_Get_Touch(FT5426_register reg, uint16_t* X, uint16_t* Y, Event_flag *Flag);
HAL_StatusTypeDef FT5426_Set_Register(FT5426_register reg, uint8_t value);
HAL_StatusTypeDef FT5426_Get_Register(FT5426_register reg, uint8_t* data);

#endif /* INC_FT5426_H_ */
