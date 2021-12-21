/**
  ******************************************************************************

  EEPROM.h Using the HAL I2C Functions
  Original source: https://github.com/controllerstech/STM32/tree/master/EEPROM_STM32

  ******************************************************************************
*/

#ifndef INC_EEPROM_H_
#define INC_EEPROM_H_

#include "stdint.h"
#include "stm32f1xx_hal.h"
#include <stdbool.h>
/**
 * @brief this is the hardware configuration for the eeprom
 */
struct i2c_eeprom_cfg{
	I2C_HandleTypeDef * hi2c; /**< The I2C bus number on which the EEPROM lies */
    uint8_t i2c_slave_addr; /**< The slave address configured in hardware */
    uint32_t timeout; /**< The maximum amount of milliseconds to wait for
                               an I2C ee_conf */
    bool write_protect;  //whether or not write protection is used on the EEPROM
    GPIO_TypeDef *  write_protect_port; //the GPIO port of the write protection pin being used
    uint16_t write_protect_pin; //the GPIO port of the write protection pin being used
	uint16_t page_size; //Bytes
	uint16_t pages;
};

typedef struct i2c_eeprom_cfg i2c_eeprom_cfg_t;

/** write the data to the EEPROM
 * @page is the number of the start page. Range from 0 to ee_conf->pages-1
 * @offset is the start byte offset in the page. Range from 0 to ee_conf->page_size-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 * @return an error if something went wrong
 */
HAL_StatusTypeDef EEPROM_Write (i2c_eeprom_cfg_t* ee_conf, uint16_t page,
		uint16_t offset, uint8_t *data, uint16_t size);

/** READ the data from the EEPROM
 * @page is the number of the start page. Range from 0 to ee_conf->pages-1
 * @offset is the start byte offset in the page. Range from 0 to ee_conf->page_size-1
 * @data is the pointer to the data to write in bytes
 * @size is the size of the data
 *  @return an error if something went wrong
 */
HAL_StatusTypeDef EEPROM_Read (i2c_eeprom_cfg_t* ee_conf, uint16_t page,
		uint16_t offset, uint8_t *data, uint16_t size);

/** Erase a page in the EEPROM Memory
 * @page is the number of page to erase
 * In order to erase multiple pages, just use this function in the for loop
 * @return an error if something went wrong
 */
HAL_StatusTypeDef EEPROM_PageErase (i2c_eeprom_cfg_t* ee_conf, uint16_t page);

/**Write the Float/Integer values to the EEPROM
 * @page is the number of the start page. Range from 0 to ee_conf->pages-1
 * @offset is the start byte offset in the page. Range from 0 to ee_conf->page_size-1
 * @data is the float/integer value that you want to write
 *  @return an error if something went wrong
 */
HAL_StatusTypeDef EEPROM_Write_NUM (i2c_eeprom_cfg_t* ee_conf,
		uint16_t page, uint16_t offset, float  fdata);

/** Reads the single Float/Integer values from the EEPROM
 * @page is the number of the start page. Range from 0 to ee_conf->pages-1
 * @offset is the start byte offset in the page. Range from 0 to ee_conf->page_size-1
 * @num is the float/integer value
 *   @return an error if something went wrong
 */
HAL_StatusTypeDef EEPROM_Read_NUM (i2c_eeprom_cfg_t* ee_conf,
		uint16_t page, uint16_t offset, float * num);

#endif /* INC_EEPROM_H_ */
