/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f7xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */
extern ADC_HandleTypeDef hadc1;
extern ADC_HandleTypeDef hadc3;

extern CRC_HandleTypeDef hcrc;

extern DMA2D_HandleTypeDef hdma2d;

extern I2C_HandleTypeDef hi2c1;
extern I2C_HandleTypeDef hi2c4;

extern JPEG_HandleTypeDef hjpeg;

extern LTDC_HandleTypeDef hltdc;

extern RNG_HandleTypeDef hrng;

extern RTC_HandleTypeDef hrtc;

extern SAI_HandleTypeDef hsai_BlockA2;
extern SAI_HandleTypeDef hsai_BlockB2;

extern SD_HandleTypeDef hsd1;

extern SPI_HandleTypeDef hspi2;
extern SPI_HandleTypeDef hspi4;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim8;

extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_uart4_tx;

extern SDRAM_HandleTypeDef hsdram1;

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define AF_AMP_MUTE_Pin GPIO_PIN_3
#define AF_AMP_MUTE_GPIO_Port GPIOE
#define AUDIO_OUT_SW_Pin GPIO_PIN_8
#define AUDIO_OUT_SW_GPIO_Port GPIOI
#define FRAM_CS_Pin GPIO_PIN_13
#define FRAM_CS_GPIO_Port GPIOC
#define KEY_IN_DOT_Pin GPIO_PIN_12
#define KEY_IN_DOT_GPIO_Port GPIOI
#define KEY_IN_DOT_EXTI_IRQn EXTI15_10_IRQn
#define KEY_IN_DASH_Pin GPIO_PIN_13
#define KEY_IN_DASH_GPIO_Port GPIOI
#define KEY_IN_DASH_EXTI_IRQn EXTI15_10_IRQn
#define RFUNIT_ALC_Pin GPIO_PIN_6
#define RFUNIT_ALC_GPIO_Port GPIOF
#define SWR_FORW_Pin GPIO_PIN_7
#define SWR_FORW_GPIO_Port GPIOF
#define SWR_BACKW_Pin GPIO_PIN_8
#define SWR_BACKW_GPIO_Port GPIOF
#define RFUNIT_THERM_Pin GPIO_PIN_9
#define RFUNIT_THERM_GPIO_Port GPIOF
#define ETH_INT_Pin GPIO_PIN_0
#define ETH_INT_GPIO_Port GPIOA
#define TIM2_CH4_TFT_LEDCTRL_DIGITAL_Pin GPIO_PIN_3
#define TIM2_CH4_TFT_LEDCTRL_DIGITAL_GPIO_Port GPIOA
#define DEBUG1_Pin GPIO_PIN_4
#define DEBUG1_GPIO_Port GPIOA
#define PHOTORESISTOR_Pin GPIO_PIN_5
#define PHOTORESISTOR_GPIO_Port GPIOA
#define ENC1_A_Pin GPIO_PIN_6
#define ENC1_A_GPIO_Port GPIOA
#define USB_OTG_HS_PSON_Pin GPIO_PIN_2
#define USB_OTG_HS_PSON_GPIO_Port GPIOB
#define RC5_Pin GPIO_PIN_4
#define RC5_GPIO_Port GPIOJ
#define RC5_EXTI_IRQn EXTI4_IRQn
#define FPGA_CLK_Pin GPIO_PIN_6
#define FPGA_CLK_GPIO_Port GPIOH
#define FPGA_SYNC_Pin GPIO_PIN_7
#define FPGA_SYNC_GPIO_Port GPIOH
#define FPGA_D0_Pin GPIO_PIN_8
#define FPGA_D0_GPIO_Port GPIOH
#define FPGA_D1_Pin GPIO_PIN_9
#define FPGA_D1_GPIO_Port GPIOH
#define FPGA_D2_Pin GPIO_PIN_10
#define FPGA_D2_GPIO_Port GPIOH
#define FPGA_D3_Pin GPIO_PIN_11
#define FPGA_D3_GPIO_Port GPIOH
#define FPGA_D4_Pin GPIO_PIN_12
#define FPGA_D4_GPIO_Port GPIOH
#define ENC2_A_Pin GPIO_PIN_12
#define ENC2_A_GPIO_Port GPIOD
#define ENC2_B_Pin GPIO_PIN_13
#define ENC2_B_GPIO_Port GPIOD
#define ENC2_SW_Pin GPIO_PIN_6
#define ENC2_SW_GPIO_Port GPIOJ
#define ENC2_SW_EXTI_IRQn EXTI9_5_IRQn
#define PWR_HOLD_Pin GPIO_PIN_7
#define PWR_HOLD_GPIO_Port GPIOJ
#define PTT_IN_Pin GPIO_PIN_8
#define PTT_IN_GPIO_Port GPIOJ
#define PTT_IN_EXTI_IRQn EXTI9_5_IRQn
#define PWR_ON_Pin GPIO_PIN_9
#define PWR_ON_GPIO_Port GPIOJ
#define PWR_ON_EXTI_IRQn EXTI9_5_IRQn
#define AUDIO_48K_CLOCK_Pin GPIO_PIN_11
#define AUDIO_48K_CLOCK_GPIO_Port GPIOJ
#define AUDIO_48K_CLOCK_EXTI_IRQn EXTI15_10_IRQn
#define ESP_INT_Pin GPIO_PIN_1
#define ESP_INT_GPIO_Port GPIOK
#define ESP_INT_EXTI_IRQn EXTI1_IRQn
#define TOUCH_INT_Pin GPIO_PIN_2
#define TOUCH_INT_GPIO_Port GPIOG
#define TOUCH_INT_EXTI_IRQn EXTI2_IRQn
#define ESP_CS_Pin GPIO_PIN_3
#define ESP_CS_GPIO_Port GPIOG
#define VS_RST_Pin GPIO_PIN_6
#define VS_RST_GPIO_Port GPIOG
#define ENC3_SW_Pin GPIO_PIN_7
#define ENC3_SW_GPIO_Port GPIOG
#define ENC3_SW_EXTI_IRQn EXTI9_5_IRQn
#define ENC3_A_Pin GPIO_PIN_6
#define ENC3_A_GPIO_Port GPIOC
#define ENC3_B_Pin GPIO_PIN_7
#define ENC3_B_GPIO_Port GPIOC
#define FPGA_D5_Pin GPIO_PIN_13
#define FPGA_D5_GPIO_Port GPIOH
#define FPGA_D6_Pin GPIO_PIN_14
#define FPGA_D6_GPIO_Port GPIOH
#define FPGA_D7_Pin GPIO_PIN_15
#define FPGA_D7_GPIO_Port GPIOH
#define TOUCH_CS_Pin GPIO_PIN_0
#define TOUCH_CS_GPIO_Port GPIOI
#define VS_DREQ_Pin GPIO_PIN_3
#define VS_DREQ_GPIO_Port GPIOD
#define VS_DREQ_EXTI_IRQn EXTI3_IRQn
#define VS_XCS_Pin GPIO_PIN_4
#define VS_XCS_GPIO_Port GPIOD
#define VS_XDCS_Pin GPIO_PIN_7
#define VS_XDCS_GPIO_Port GPIOD
#define AD1_CS_Pin GPIO_PIN_12
#define AD1_CS_GPIO_Port GPIOJ
#define SDMMC1_CD_Pin GPIO_PIN_15
#define SDMMC1_CD_GPIO_Port GPIOJ
#define RFUNIT_RCLK_Pin GPIO_PIN_9
#define RFUNIT_RCLK_GPIO_Port GPIOG
#define RFUNIT_CLK_Pin GPIO_PIN_13
#define RFUNIT_CLK_GPIO_Port GPIOG
#define RFUNIT_DATA_Pin GPIO_PIN_3
#define RFUNIT_DATA_GPIO_Port GPIOK
#define RFUNIT_OE_Pin GPIO_PIN_7
#define RFUNIT_OE_GPIO_Port GPIOK
#define ENC1_B_Pin GPIO_PIN_5
#define ENC1_B_GPIO_Port GPIOB
#define PHY_ETH_RST_Pin GPIO_PIN_4
#define PHY_ETH_RST_GPIO_Port GPIOI
#define HEARTBEAT_LED_Pin GPIO_PIN_6
#define HEARTBEAT_LED_GPIO_Port GPIOI

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
