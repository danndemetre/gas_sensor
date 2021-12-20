/*
 * lm60.c
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */
#include "components/lm60.h"
#include "components/ads1115.h"
#include "stdlib.h"

lm60_cfg_t lm60_init(){
	ads1115_i2c_conf_t ads1115_i2c_conf = (ads1115_i2c_conf_t){
			.hi2c = hw_conf_lm60_ads1115().ads1115_hi2c,
			.i2c_slave_addr =hw_conf_lm60_ads1115().ads1115_i2c_slave_addr,
			.timeout = hw_conf_lm60_ads1115().ads1115_timeout,
	};
	ads1115_config_t ads1115_reg_config = 	 (ads1115_config_t){
			.os = ADS1115_OS,
			.pin = hw_conf_lm60_ads1115().ads1115_pin,
			.gain = ADS1115_1_024V,
			.mode = ADS1115_SINGLE_SHOT,
			.data_rate = ADS1115_DEF_SPS,
			.comp = ADS1115_DEF_COMP,
			.polarity = ADS1115_DEF_POL,
			.latch = ADS1115_DEF_LATCH,
			.queue = ADS1115_COMP_DISABLE,
	};

	return (lm60_cfg_t){
		.ads_i2c_conf = &ads1115_i2c_conf,
		.ads_reg_conf = &ads1115_reg_config,
	};
}

HAL_StatusTypeDef lm60_get_temp(lm60_cfg_t* lm60_conf, float * temp_c){
	HAL_StatusTypeDef err = ads1115_write_cfg(lm60_conf->ads_i2c_conf,  lm60_conf->ads_reg_conf);
	int16_t mv;
	if (err == HAL_OK){
		err = ads1115_read_adc_millivolts(lm60_conf->ads_i2c_conf, lm60_conf->ads_reg_conf, &mv);
		if( err == HAL_OK){
			* temp_c = (mv - 424) / 6.25;
		}
	}
	return err;
}
