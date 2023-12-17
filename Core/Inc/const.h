#ifndef INC_CONST_H_
#define INC_CONST_H_

#include "debug.h"

//----- DEFINES -----
//Application settings
#define VERSION                        "0.10"
#define RELEASE                        //Release version
#define APPNAME                        "WebRadio" //max 15 characters
#if defined DEBUG
# define APPVERSION                    VERSION"a"
#elif defined RELEASE
# define APPVERSION                    VERSION
#else
# define APPVERSION                    VERSION"*"
#endif

//Check hardware config
#define STM32F767
#define LCD_SSD1289

#if defined STM32F207
# define STM_NAME "STM32F207VET6"
#elif defined STM32F767
# define STM_NAME "STM32F767BIT6"
#elif defined STM32F103ZET6
# define STM_NAME "STM32F103ZET6"
#else
# warning "STM32F Rev not defined"
#endif

#if defined LCD_SSD1289
#define LCD_NAME "SSD1289"
# define LCD_WIDTH                     (320)
# define LCD_HEIGHT                    (240)
#elif defined LCD_L2F50                  //LCD
# define LCD_NAME "S65-L2F50"
#elif defined LCD_LPH88
# define LCD_NAME "S65-LPH88"
#elif defined LCD_LS020
# define LCD_NAME "S65-LS020"
#elif defined LCD_MIO283QT1
# define LCD_NAME "MIO283QT1"
#elif defined LCD_MIO283QT2
# define LCD_NAME "MIO283QT2"
#elif defined LCD_ILI9325
# define LCD_NAME "LCD_ILI9325"
#else
# warning "LCD not defined"
#endif



#endif /* INC_CONST_H_ */
