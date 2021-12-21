/*
 * lm60.c
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */
#include "components/lm60.h"
#include "components/ads1115.h"
#include "stdlib.h"

#include <string.h>
#include <stdio.h>
#include "hw_config.h"

HAL_StatusTypeDef lm60_get_temp(lm60_cfg_t* lm60_conf, float * temp_c){
	int16_t mv;
	HAL_StatusTypeDef err = ads1115_write_cfg(&lm60_conf->ads_i2c_conf,  &lm60_conf->ads_reg_conf);

	if (err == HAL_OK){
		err = ads1115_read_adc_millivolts(&lm60_conf->ads_i2c_conf, &lm60_conf->ads_reg_conf, &mv);
		if( err == HAL_OK){
			* temp_c = (mv - 424) / 6.25;
		}
	}
	return err;
}
