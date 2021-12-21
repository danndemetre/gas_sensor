/*
 * hw_config.h
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */

#ifndef INC_HW_CONFIG_H_
#define INC_HW_CONFIG_H_

#include "components/ads1115.h"
#include "components/lm60.h"
#include "components/EEPROM.h"

/**
 * @brief The I2C port 2 HAL struct
 */
I2C_HandleTypeDef hi2c2;

/**
 * @brief The UART port 1 HAL struct
 */
UART_HandleTypeDef huart1;

/**
 *
 * @return the hardware configuration for the lm60
 */
lm60_cfg_t hw_conf_lm60();

/**
 * @return the hardware configuration for the EEPROM
 */
i2c_eeprom_cfg_t hw_conf_m24c64_w();

/**
 * @return HW configuration for the LM60 temperature sensor
 */
struct  hw_conf_lm60 hw_conf_lm60_ads1115();

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void);

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
void MX_USART1_UART_Init(void);

/**
 * @brief Setup the I2C2 port
 */
void MX_I2C2_Init(void);

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void);

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void);

#endif /* INC_HW_CONFIG_H_ */
