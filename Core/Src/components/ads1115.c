/*
 * ads1115.c
 *
 *  Created on: Dec 19, 2021
 *      Author: dann
 */

#include "hw_config.h"
#include "components/ads1115.h"
#include <stdlib.h>
#include <stdint.h>

ads1115_raw_conf_t ads1115_encode_cfg(const ads1115_config_t * conf){
	ads1115_raw_conf_t raw_conf = 0x0000;
	raw_conf |= ((uint16_t)conf->os & ADS1115_OS_BIT_MASK) << ADS1115_OS_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->pin & ADS1115_PIN_BIT_MASK) << ADS1115_PIN_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->gain & ADS1115_PGA_BIT_MASK) << ADS1115_PGA_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->mode & ADS1115_MODE_BIT_MASK) << ADS1115_MODE_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->data_rate & ADS1115_DR_BIT_MASK) << ADS1115_DR_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->comp & ADS1115_COMP_BIT_MASK) << ADS1115_COMP_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->polarity & ADS1115_POL_BIT_MASK) << ADS1115_POL_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->latch & ADS1115_LATCH_BIT_MASK) << ADS1115_LATCH_BIT_OFFSET;
	raw_conf |= ((uint16_t)conf->que & ADS1115_QUEUE_BIT_MASK) << ADS1115_QUEUE_BIT_OFFSET;
	return raw_conf;
}

ads1115_config_t   ads1115_decode_cfg(const ads1115_raw_conf_t  raw_conf){
	ads1115_config_t conf;
	conf.os = (raw_conf >> ADS1115_OS_BIT_OFFSET) & ADS1115_OS_BIT_MASK;
	conf.pin = (raw_conf >> ADS1115_PIN_BIT_OFFSET) & ADS1115_PIN_BIT_MASK;
	conf.gain = (raw_conf >> ADS1115_PGA_BIT_OFFSET) & ADS1115_PGA_BIT_MASK;
	conf.mode = (raw_conf >> ADS1115_MODE_BIT_OFFSET) & ADS1115_MODE_BIT_MASK;
	conf.data_rate = (raw_conf >> ADS1115_DR_BIT_OFFSET) & ADS1115_DR_BIT_MASK;
	conf.polarity = (raw_conf >> ADS1115_POL_BIT_OFFSET) & ADS1115_POL_BIT_MASK;
	conf.latch = (raw_conf >> ADS1115_LATCH_BIT_OFFSET) & ADS1115_LATCH_BIT_MASK;
	conf.que = (raw_conf >> ADS1115_QUEUE_BIT_OFFSET) & ADS1115_QUEUE_BIT_MASK;
	return conf;
}

HAL_StatusTypeDef ads1115_read_cfg(const ads1115_i2c_conf_t* i2c_conf, ads1115_config_t * conf)
{
    ads1115_raw_conf_t raw_conf ;
    HAL_StatusTypeDef err;
    uint8_t ads_reg_set[1] = {ADS1115_CONFIGURATION_REG};
    uint8_t raw_conf_eight_bit[2];

    err = HAL_I2C_Master_Transmit(&hi2c1, (i2c_conf->i2c_slave_addr << 1) | I2C_WRITE,  ads_reg_set, 1, i2c_conf->timeout);

    if (err == HAL_OK){
    	err = HAL_I2C_Master_Receive(i2c_conf->hi2c, (i2c_conf->i2c_slave_addr << 1) | I2C_READ, raw_conf_eight_bit, 2, i2c_conf->timeout);
    }
    raw_conf = ((ads1115_raw_conf_t)raw_conf_eight_bit[0] << 8 )| raw_conf_eight_bit[1];
    *conf = ads1115_decode_cfg(raw_conf);
    return err;
}

HAL_StatusTypeDef ads1115_write_cfg(const ads1115_i2c_conf_t* i2c_conf, const ads1115_config_t * conf)

{
	 HAL_StatusTypeDef err;
    ads1115_raw_conf_t raw_conf = ads1115_encode_cfg(conf);
    uint8_t i2c_buf[3] = { ADS1115_CONFIGURATION_REG, raw_conf >> 8, raw_conf & 0xFF};
    err = HAL_I2C_Master_Transmit(i2c_conf->hi2c, ( i2c_conf->i2c_slave_addr << 1) | I2C_WRITE,
    		i2c_buf, 3, i2c_conf->timeout);

   return err;
}

HAL_StatusTypeDef __ads1115_convert_raw_voltage(const ads1115_config_t * conf,
		const int16_t raw_value, int32_t* converted_value)

{
    double buf = ((double)raw_value / 32.768);
    HAL_StatusTypeDef err = HAL_OK;
    switch (conf->gain) {
		case ADS1115_6_144V: buf = buf * 6144; break;
		case ADS1115_4_096V: buf = buf * 4096; break;
		case ADS1115_2_048V: buf = buf * 2048; break;
		case ADS1115_1_024V: buf = buf * 1024; break;
		case ADS1115_0_512V: buf = buf * 512; break;
		case ADS1115_0_256V: buf = buf * 256; break;
		default: return HAL_ERROR;
    }

    *converted_value = (uint32_t)buf;

    return err;
}

HAL_StatusTypeDef __ads1115_read_to_microvolts(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value, const uint8_t dev_register)
{
	uint8_t ads_reg_set[1] = {dev_register};
	int16_t raw_value = 0;
	HAL_StatusTypeDef err = HAL_OK;

	if(conf->mode == ADS1115_SINGLE_SHOT){
		ads1115_config_t cnf_cpy = *conf;
		cnf_cpy.os = ADS1115_IDLE_OR_START;
		ads1115_write_cfg(i2c_conf, &cnf_cpy);
		cnf_cpy.mode = ADS1115_SINGLE_SHOT;
	}

	if (err == HAL_OK){
		uint8_t conv_res_eight_bit[2];
		err = HAL_I2C_Master_Transmit(i2c_conf->hi2c,  (i2c_conf->i2c_slave_addr << 1) | I2C_WRITE,
				ads_reg_set, 1, i2c_conf->timeout);
		if (err == HAL_OK){
			err = HAL_I2C_Master_Receive(i2c_conf->hi2c, (i2c_conf->i2c_slave_addr << 1),
					conv_res_eight_bit, 2, i2c_conf->timeout);
		}
		raw_value = ((int16_t)conv_res_eight_bit[0] << 8 )| conv_res_eight_bit[1];
		if (err == HAL_OK) {
			err = __ads1115_convert_raw_voltage(conf, raw_value, uv_value);
		}
	}
	return err;
}

HAL_StatusTypeDef ads1115_read_adc_microvolts(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv)
{
	return __ads1115_read_to_microvolts(i2c_conf, conf, uv, ADS1115_CONVERSION_REG);
}

HAL_StatusTypeDef ads1115_read_low_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value){
	return __ads1115_read_to_microvolts(i2c_conf, conf, uv_value, ADS1115_LOW_THRES_REG);
}

HAL_StatusTypeDef ads1115_read_high_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value){
	return __ads1115_read_to_microvolts(i2c_conf, conf, uv_value, ADS1115_HIGH_THRES_REG);
}

HAL_StatusTypeDef __ads1115_write_to_microvolts(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value, const uint8_t  dev_register)
{
    HAL_StatusTypeDef err;
    err = ads1115_write_cfg(i2c_conf, conf);
	if (err != HAL_OK){
		return err;
	}

	int16_t gain;
	uint16_t discrete_steps = 32768;

    switch (conf->gain) {
		case ADS1115_6_144V: gain = 6144; break;
		case ADS1115_4_096V: gain =  4096; break;
		case ADS1115_2_048V: gain = 2048; break;
		case ADS1115_1_024V: gain = 1024; break;
		case ADS1115_0_512V: gain = 512; break;
		case ADS1115_0_256V: gain = 256; break;
		default: return HAL_ERROR;
    }
    ads1115_raw_conf_t raw_value = (int16_t) (( (*uv_value / gain) * discrete_steps) / 1000);
    uint8_t i2c_buf[3] = { dev_register, raw_value>> 8, raw_value & 0xFF};
    err = HAL_I2C_Master_Transmit(i2c_conf->hi2c,  (i2c_conf->i2c_slave_addr << 1) | I2C_WRITE,  i2c_buf, 3, i2c_conf->timeout);
    return err;
}

HAL_StatusTypeDef ads1115_write_low_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value){
	return __ads1115_write_to_microvolts( i2c_conf,  conf,  uv_value, ADS1115_LOW_THRES_REG);
}

HAL_StatusTypeDef ads1115_write_high_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value){
	return __ads1115_write_to_microvolts( i2c_conf,  conf,  uv_value, ADS1115_HIGH_THRES_REG);
}
