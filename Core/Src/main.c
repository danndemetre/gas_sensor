
#include "main.h"
#include "hw_config.h"

#include <string.h>
#include <stdio.h>
/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  HAL_Init();

  SystemClock_Config();

  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_USART1_UART_Init();
  MX_CAN_Init();
  MX_I2C1_Init();

  float temp_c = 0;
  lm60_cfg_t lm60 = hw_conf_lm60();

  while (1){
	  lm60_get_temp(&lm60, &temp_c);
	 HAL_Delay(1000);
  }
}
