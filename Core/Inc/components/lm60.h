/*
 * lm60.h
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */

#ifndef INC_COMPONENTS_LM60_H_
#define INC_COMPONENTS_LM60_H_

#include "components/ads1115.h"

/**
 * @brief lm60 i2c_conf consisting of ads i2c_conf and config
 */
struct lm60_config {
	ads1115_i2c_conf_t  ads_i2c_conf;
	ads1115_config_t  ads_reg_conf;
};

typedef struct lm60_config lm60_cfg_t;

/**
 * @brief Start the temperature conversion process, if running in any other mode than
 * 			single-shot non-blocking than it is not necessary to call this function
 * @param lm60_cfg
 * @param temp_c
 * @return An error if something went wrong
 */
HAL_StatusTypeDef lm60_start_temp_reading(const lm60_cfg_t* lm60_cfg);

/**
 * @brief  Gets the temperature of the LM60 attached to the ADS1115
 * @param i2c_conf
 * @param conf
 * @param temp_c
 * @return The temperature from the lm60 connected to the ads1115
 */
HAL_StatusTypeDef lm60_get_temp(const lm60_cfg_t* lm60_cfg, float * temp_c);

#endif /* INC_COMPONENTS_LM60_H_ */
