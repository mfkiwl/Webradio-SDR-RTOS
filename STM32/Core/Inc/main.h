/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2021 STMicroelectronics.
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
#include "stm32f2xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

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
#define LCD_RESET_Pin GPIO_PIN_2
#define LCD_RESET_GPIO_Port GPIOE
#define VS_XDCS_Pin GPIO_PIN_3
#define VS_XDCS_GPIO_Port GPIOE
#define VS_XCS_Pin GPIO_PIN_4
#define VS_XCS_GPIO_Port GPIOE
#define VS_DREQ_Pin GPIO_PIN_5
#define VS_DREQ_GPIO_Port GPIOE
#define VS_RST_Pin GPIO_PIN_6
#define VS_RST_GPIO_Port GPIOE
#define PHY_ETH_RST_Pin GPIO_PIN_0
#define PHY_ETH_RST_GPIO_Port GPIOC
#define PHY_ETH_MDC_Pin GPIO_PIN_1
#define PHY_ETH_MDC_GPIO_Port GPIOC
#define AUDIO_CHAN_R_Pin GPIO_PIN_2
#define AUDIO_CHAN_R_GPIO_Port GPIOC
#define AUDIO_CHAN_L_Pin GPIO_PIN_3
#define AUDIO_CHAN_L_GPIO_Port GPIOC
#define PHY_ETH_INT_Pin GPIO_PIN_0
#define PHY_ETH_INT_GPIO_Port GPIOA
#define PHY_ETH_RMII_REF_CLK_Pin GPIO_PIN_1
#define PHY_ETH_RMII_REF_CLK_GPIO_Port GPIOA
#define PHY_ETH_MDIO_Pin GPIO_PIN_2
#define PHY_ETH_MDIO_GPIO_Port GPIOA
#define SD_CARD_DETECT_Pin GPIO_PIN_6
#define SD_CARD_DETECT_GPIO_Port GPIOA
#define PHY_ETH_RMII_CRS_DV_Pin GPIO_PIN_7
#define PHY_ETH_RMII_CRS_DV_GPIO_Port GPIOA
#define PHY_ETH_RMII_RXD0_Pin GPIO_PIN_4
#define PHY_ETH_RMII_RXD0_GPIO_Port GPIOC
#define PHY_ETH_RMII_RXD1_Pin GPIO_PIN_5
#define PHY_ETH_RMII_RXD1_GPIO_Port GPIOC
#define IR_RC5_DATA_Pin GPIO_PIN_7
#define IR_RC5_DATA_GPIO_Port GPIOE
#define ALARM_AMP_ON_Pin GPIO_PIN_8
#define ALARM_AMP_ON_GPIO_Port GPIOE
#define TIM1_CH1_ENCODER_PH1_Pin GPIO_PIN_9
#define TIM1_CH1_ENCODER_PH1_GPIO_Port GPIOE
#define ENCODER_PUSH_BUTTON_Pin GPIO_PIN_10
#define ENCODER_PUSH_BUTTON_GPIO_Port GPIOE
#define TIM1_CH1_ENCODER_PH2_Pin GPIO_PIN_11
#define TIM1_CH1_ENCODER_PH2_GPIO_Port GPIOE
#define FRAM_CS_Pin GPIO_PIN_12
#define FRAM_CS_GPIO_Port GPIOE
#define SD_CARD_CS_Pin GPIO_PIN_13
#define SD_CARD_CS_GPIO_Port GPIOE
#define LCD_TP_CS_Pin GPIO_PIN_14
#define LCD_TP_CS_GPIO_Port GPIOE
#define LCD_TP_IRQ_Pin GPIO_PIN_15
#define LCD_TP_IRQ_GPIO_Port GPIOE
#define PHY_ETH_RMII_TX_EN_Pin GPIO_PIN_11
#define PHY_ETH_RMII_TX_EN_GPIO_Port GPIOB
#define PHY_ETH_RMII_TXD0_Pin GPIO_PIN_12
#define PHY_ETH_RMII_TXD0_GPIO_Port GPIOB
#define PHY_ETH_RMII_TXD1_Pin GPIO_PIN_13
#define PHY_ETH_RMII_TXD1_GPIO_Port GPIOB
#define LCD_DATA_8_Pin GPIO_PIN_8
#define LCD_DATA_8_GPIO_Port GPIOD
#define LCD_DATA_9_Pin GPIO_PIN_9
#define LCD_DATA_9_GPIO_Port GPIOD
#define LCD_DATA_10_Pin GPIO_PIN_10
#define LCD_DATA_10_GPIO_Port GPIOD
#define LCD_DATA_11_Pin GPIO_PIN_11
#define LCD_DATA_11_GPIO_Port GPIOD
#define LCD_DATA_12_Pin GPIO_PIN_12
#define LCD_DATA_12_GPIO_Port GPIOD
#define LCD_DATA_13_Pin GPIO_PIN_13
#define LCD_DATA_13_GPIO_Port GPIOD
#define LCD_DATA_14_Pin GPIO_PIN_14
#define LCD_DATA_14_GPIO_Port GPIOD
#define LCD_DATA_15_Pin GPIO_PIN_15
#define LCD_DATA_15_GPIO_Port GPIOD
#define ESP_IRQ_Pin GPIO_PIN_8
#define ESP_IRQ_GPIO_Port GPIOC
#define ESP_CS_Pin GPIO_PIN_9
#define ESP_CS_GPIO_Port GPIOC
#define ESP_RESET_Pin GPIO_PIN_8
#define ESP_RESET_GPIO_Port GPIOA
#define USB_DM_Pin GPIO_PIN_11
#define USB_DM_GPIO_Port GPIOA
#define USB_DP_Pin GPIO_PIN_12
#define USB_DP_GPIO_Port GPIOA
#define JTMS_Pin GPIO_PIN_13
#define JTMS_GPIO_Port GPIOA
#define JTCK_Pin GPIO_PIN_14
#define JTCK_GPIO_Port GPIOA
#define JTDI_Pin GPIO_PIN_15
#define JTDI_GPIO_Port GPIOA
#define LCD_DATA_0_Pin GPIO_PIN_0
#define LCD_DATA_0_GPIO_Port GPIOD
#define LCD_DATA_1_Pin GPIO_PIN_1
#define LCD_DATA_1_GPIO_Port GPIOD
#define LCD_DATA_2_Pin GPIO_PIN_2
#define LCD_DATA_2_GPIO_Port GPIOD
#define LCD_DATA_3_Pin GPIO_PIN_3
#define LCD_DATA_3_GPIO_Port GPIOD
#define LCD_DATA_4_Pin GPIO_PIN_4
#define LCD_DATA_4_GPIO_Port GPIOD
#define LCD_DATA_5_Pin GPIO_PIN_5
#define LCD_DATA_5_GPIO_Port GPIOD
#define LCD_DATA_6_Pin GPIO_PIN_6
#define LCD_DATA_6_GPIO_Port GPIOD
#define LCD_DATA_7_Pin GPIO_PIN_7
#define LCD_DATA_7_GPIO_Port GPIOD
#define JTDO_Pin GPIO_PIN_3
#define JTDO_GPIO_Port GPIOB
#define JTRST_Pin GPIO_PIN_4
#define JTRST_GPIO_Port GPIOB
#define LCD_CS_Pin GPIO_PIN_5
#define LCD_CS_GPIO_Port GPIOB
#define LCD_RS_Pin GPIO_PIN_8
#define LCD_RS_GPIO_Port GPIOB
#define LCD_BL_CNT_Pin GPIO_PIN_9
#define LCD_BL_CNT_GPIO_Port GPIOB
#define LCD_WR_Pin GPIO_PIN_0
#define LCD_WR_GPIO_Port GPIOE
#define LCD_RD_Pin GPIO_PIN_1
#define LCD_RD_GPIO_Port GPIOE
/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
