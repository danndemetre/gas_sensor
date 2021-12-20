/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
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
#include "hw_config.h"
#include "components/lm60.h"

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();
  SystemClock_Config();
  MX_GPIO_Init();
  MX_USART1_UART_Init();
  MX_I2C2_Init();

 float temp_c = 0;
 lm60_cfg_t  lm60 = hw_conf_lm60();
 //hw_i2c_eeprom_cfg_t m24c64_eeprom = hw_conf_m24c64_w();

while (1)
  {
	  lm60_get_temp(&lm60, &temp_c);
	  HAL_Delay(1000);
  }
}
