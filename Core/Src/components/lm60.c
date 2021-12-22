/*
 * lm60.c
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */
#include "components/lm60.h"
#include "components/ads1115.h"


HAL_StatusTypeDef lm60_start_temp_reading(const lm60_cfg_t* lm60_conf){
	return ads1115_start_reading(&lm60_conf->ads_i2c_conf,
			&lm60_conf->ads_reg_conf);
}

HAL_StatusTypeDef lm60_get_temp(const lm60_cfg_t* lm60_conf, float * temp_c){
	int32_t uv;
	HAL_StatusTypeDef err;
     err = ads1115_read_adc_microvolts(&lm60_conf->ads_i2c_conf, &lm60_conf->ads_reg_conf, &uv);
	 if( err == HAL_OK){
		* temp_c = (uv - 424000) / 6250.0;
	}
	return err;
}
