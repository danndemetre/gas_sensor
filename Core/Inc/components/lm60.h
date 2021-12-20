/*
 * lm60.h
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */

#ifndef INC_COMPONENTS_LM60_H_
#define INC_COMPONENTS_LM60_H_

#include "components/ads1115.h"

/*
 * @brief the hw_configuration struct for the lm60
 */
struct hw_conf_lm60 {
	I2C_HandleTypeDef * ads1115_hi2c; /*STM32 I2C number of the ADS1115 that the LM60 is connected to*/
	uint8_t ads1115_i2c_slave_addr; /*ADS1115 address of the ADS1115 that the LM60 is connected to*/
	uint32_t ads1115_timeout; /*Associated ADS1115 timeout for LM60 operations*/
	enum ads1115_pin ads1115_pin; /*Input channel for the LM60 to the ADS1115 */
};

typedef struct hw_conf_lm60 hw_conf_lm60_t;

/**
 * @brief lm60 i2c_conf consisting of ads i2c_conf and config
 */
struct lm60_config {
	ads1115_i2c_conf_t * ads_i2c_conf;
	ads1115_config_t * ads_reg_conf;
};

typedef struct lm60_config lm60_cfg_t;

/**
 * @brief  Gets the temperature of the LM60 attached to the ADS1115
 * @param i2c_conf
 * @param conf
 * @param temp_c
 * @return The temperature from the lm60 connected to the ads1115
 */
HAL_StatusTypeDef lm60_get_temp(lm60_cfg_t* lm60_cfg, float * temp_c);

#endif /* INC_COMPONENTS_LM60_H_ */
