
/**
  ******************************************************************************

  EEPROM.c Using the HAL I2C Functions
  Original source: https://github.com/controllerstech/STM32/tree/master/EEPROM_STM32

******************************************************************************/

#include "components/EEPROM.h"
#include "math.h"
#include "string.h"

uint8_t bytes_temp[4];

// function to determine the remaining bytes
uint16_t bytestowrite (const i2c_eeprom_cfg_t* ee_conf, uint16_t size, const uint16_t offset)
{
	if ((size+offset)<ee_conf->page_size) return size;
	else return ee_conf->page_size-offset;
}

HAL_StatusTypeDef EEPROM_Write (const i2c_eeprom_cfg_t* ee_conf, const uint16_t page,
		uint16_t offset,  uint8_t *data, uint16_t size)
{
	HAL_StatusTypeDef err = HAL_OK;
	// Find out the number of bit, where the page addressing starts
	int paddrposition = log(ee_conf->page_size)/log(2);
	// calculate the start page and the end page
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/ee_conf->page_size);

	// number of pages to be written
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	// write the data
	int i = 0;
	while( (i<numofpages) && (err == HAL_OK))
	{
		/* calculate the address of the memory location
		 * Here we add the page address with the byte address
		 */
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(ee_conf, size, offset);  // calculate the remaining bytes to be written
		if(ee_conf->write_protect){
			HAL_GPIO_WritePin (ee_conf->write_protect_port, ee_conf->write_protect_pin, GPIO_PIN_RESET);
		}
		// write the data to the EEPROM
		err = HAL_I2C_Mem_Write(ee_conf->hi2c, ee_conf->i2c_slave_addr, MemAddress, 2,
				&data[pos], bytesremaining, ee_conf->timeout);

		startPage += 1;  // increment the page, so that a new page address can be selected for further write
		offset=0;   // since we will be writing to a new page, so offset will be 0
		size = size-bytesremaining;  // reduce the size of the bytes
		pos += bytesremaining;  // update the position for the data buffer

		HAL_Delay (5);  // Write cycle delay (5ms)
		if(ee_conf->write_protect){
			HAL_GPIO_WritePin (ee_conf->write_protect_port, ee_conf->write_protect_pin, GPIO_PIN_SET);
		}
		i++;
	}
	return err;
}

void float2Bytes(uint8_t * ftoa_bytes_temp,float float_variable)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    thing.a = float_variable;

    for (uint8_t i = 0; i < 4; i++) {
      ftoa_bytes_temp[i] = thing.bytes[i];
    }
}

float Bytes2float(uint8_t * ftoa_bytes_temp)
{
    union {
      float a;
      uint8_t bytes[4];
    } thing;

    for (uint8_t i = 0; i < 4; i++) {
    	thing.bytes[i] = ftoa_bytes_temp[i];
    }

   float float_variable =  thing.a;
   return float_variable;
}

HAL_StatusTypeDef EEPROM_Write_NUM (const i2c_eeprom_cfg_t* ee_conf,
		const uint16_t page, const uint16_t offset, float data)
{
	HAL_StatusTypeDef err = HAL_OK;
	float2Bytes(bytes_temp, data);
	err = EEPROM_Write(ee_conf, page, offset, bytes_temp, 4);
	return err;
}

HAL_StatusTypeDef EEPROM_Read_NUM (const i2c_eeprom_cfg_t* ee_conf,
		const uint16_t page, const uint16_t offset, float * num)
{
	HAL_StatusTypeDef err = HAL_OK;
	uint8_t buffer[4];
	err = EEPROM_Read(ee_conf, page, offset, buffer, 4);
	if(err == HAL_OK){
		*num = Bytes2float(buffer);
	}
	return err;
}

HAL_StatusTypeDef EEPROM_Read (const i2c_eeprom_cfg_t* ee_conf, const uint16_t page,  uint16_t offset,
		uint8_t *data,  uint16_t size)
{
	HAL_StatusTypeDef err = HAL_OK;
	int paddrposition = log(ee_conf->page_size)/log(2);
	uint16_t startPage = page;
	uint16_t endPage = page + ((size+offset)/ee_conf->page_size);
	uint16_t numofpages = (endPage-startPage) + 1;
	uint16_t pos=0;

	int i = 0;
	while( (i<numofpages) && (err == HAL_OK))
	{
		uint16_t MemAddress = startPage<<paddrposition | offset;
		uint16_t bytesremaining = bytestowrite(ee_conf, size, offset);
		err = HAL_I2C_Mem_Read(ee_conf->hi2c, ee_conf->i2c_slave_addr, MemAddress,
				2, &data[pos], bytesremaining, ee_conf->timeout);
		startPage += 1;
		offset=0;
		size = size-bytesremaining;
		pos += bytesremaining;
		i++;
	}
	return err;
}

HAL_StatusTypeDef EEPROM_PageErase (const i2c_eeprom_cfg_t* ee_conf, const uint16_t page)
{
	HAL_StatusTypeDef err = HAL_OK;
	// calculate the memory address based on the page number
	int paddrposition = log(ee_conf->page_size)/log(2);
	uint16_t MemAddress = page<<paddrposition;

	// create a buffer to store the reset values
	uint8_t data[ee_conf->page_size];
	memset(data,0xff,ee_conf->page_size);

	// write the data to the EEPROM
	if(ee_conf->write_protect){
		HAL_GPIO_WritePin (ee_conf->write_protect_port, ee_conf->write_protect_pin, GPIO_PIN_RESET);
	}
	err = HAL_I2C_Mem_Write(ee_conf->hi2c, ee_conf->i2c_slave_addr,
			MemAddress, 2, data, ee_conf->page_size, ee_conf->timeout);

	HAL_Delay (5);  // write cycle delay 

	if(ee_conf->write_protect){
		HAL_GPIO_WritePin (ee_conf->write_protect_port, ee_conf->write_protect_pin, GPIO_PIN_SET);
	}
	return err;
}
