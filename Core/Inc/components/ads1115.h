#ifndef ADS1115_H
#define ADS1115_H
/**
 * @file ads1115.h
 *
 * @brief Driver for the ADS1115 I2C chip.
 *
 */
#include "stm32f1xx_hal.h"
#include <stdint.h>
#include <stdbool.h>

#define ADS1115BUS_ADDRESS_GND  0x48
#define ADS1115_BUS_ADDRESS_VDD 0x49
#define ADS1115_BUS_ADDRESS_SCL  0x4B

/**
 * @brief the registers available on the ADS1115
 */
enum ads1115_reg {
	ADS1115_CONVERSION_REG = 0x00, /* The ADS1115 Register where measured values are stored */
	ADS1115_CONFIGURATION_REG = 0x01,/* The ADS1115 Register where commands are written to configure readings */
	 ADS1115_LOW_THRES_REG = 0x02, 	/* Low water mark to signal an alert */
	 ADS1115_HIGH_THRES_REG = 0x03, 	/* High water mark to signal an alert */
};

/**
 * @brief A raw read configuration
 */
typedef uint16_t ads1115_raw_conf_t;

/**
 * @brief The ADS1115  parameters
 */
struct ads1115_i2c_conf {
	I2C_HandleTypeDef * hi2c; /**< The I2C bus number on which the ADS lies */
    uint8_t i2c_slave_addr; /**< The slave address configured in hardware */
    uint32_t timeout; /**< The maximum amount of milliseconds to wait for
                               an I2C i2c_conf */
    bool single_shot_block; /*whether to block everything and wait for reading to be processed
     	 	 	 	 	 	 	 	 	 	 when in single shot mode ~15ms. If you set to false you need to go start the reading, then come back
     	 	 	 	 	 	 	 	 	 	 later to read it. */
};

/**
 * @brief A handle to an ADC i2c_conf
 */
typedef struct ads1115_i2c_conf ads1115_i2c_conf_t;

/**
 * @brief The various bit offsets for the different parameters
 */
enum ads1115_cfg_bit_offset {
	ADS1115_OS_BIT_OFFSET      = 15,
	ADS1115_PIN_BIT_OFFSET     = 12,
	ADS1115_PGA_BIT_OFFSET    =   9,
	ADS1115_MODE_BIT_OFFSET =  8,
	ADS1115_DR_BIT_OFFSET       =  5,
	ADS1115_COMP_BIT_OFFSET =  4,
	ADS1115_POL_BIT_OFFSET     =  3,
	ADS1115_LATCH_BIT_OFFSET  = 2,
	ADS1115_QUEUE_BIT_OFFSET = 0,
};

/*
 * @brief The various bit offsets for the different parameters
 */
enum ads1115_cfg_bit_masks {
	ADS1115_OS_BIT_MASK      = 0x1,
	ADS1115_PIN_BIT_MASK     = 0b111,
	ADS1115_PGA_BIT_MASK    = 0b111,
	ADS1115_MODE_BIT_MASK = 0b1,
	ADS1115_DR_BIT_MASK       = 0b111,
	ADS1115_COMP_BIT_MASK = 0b1,
	ADS1115_POL_BIT_MASK     = 0b1,
	ADS1115_LATCH_BIT_MASK  = 0b1,
	ADS1115_QUEUE_BIT_MASK = 0b11,
};

/**
 * @brief The ADC Operational Status
 */
enum ads1115_os {
    ADS1115_ACTIVE                = 0b0,
    ADS1115_IDLE_OR_START = 0b1,
    ADS1115_OS                       = 0b0,
};

/**
 * @brief The ADC pin select/configuration
 */
enum ads1115_pin {
    ADS1115_AIN0_COMP_AIN1 = 0b000,
	ADS1115_AIN0_COMP_AIN3 = 0b001,
	ADS1115_AIN1_COMP_AIN3 = 0b010,
	ADS1115_AIN2_COMP_AIN3 = 0b011,
	ADS1115_AIN0_COMP_GND = 0b100,
	ADS1115_AIN1_COMP_GND = 0b101,
	ADS1115_AIN2_COMP_GND = 0b110,
	ADS1115_AIN3_COMP_GND = 0b111
};

/**
 * @brief The ADC gain configuration
 */
enum ads1115_gain {
    ADS1115_6_144V = 0b000,
	ADS1115_4_096V = 0b001,
	ADS1115_2_048V = 0b010,
	ADS1115_1_024V = 0b011,
	ADS1115_0_512V = 0b100,
	ADS1115_0_256V = 0b101,
};

/**
 * @brief The ADC mode select
 */
enum ads1115_mode {
    ADS1115_CONTINUOUS = 0x0,
    ADS1115_SINGLE_SHOT = 0b1,
};

/**
 * @brief The ADC data rate configuration
 */
enum ads1115_data_rate {
	ADS1115_8_SPS      = 0b000,
	ADS1115_16_SPS    = 0b001,
	ADS1115_32_SPS    = 0b010,
	ADS1115_64_SPS    = 0b011,
	ADS1115_128_SPS  = 0b100,
	ADS1115_250_SPS  = 0b101,
	ADS1115_475_SPS  = 0b110,
	ADS1115_860_SPS  = 0b111,
	ADS1115_DEF_SPS  = 0b100,
};

/**
 * @brief The ADC comparator type
 */
enum ads1115_comparator {
	ADS1115_TRADITIONAL = 0b0,
	ADS1115_WINDOW      = 0b1,
	ADS1115_DEF_COMP = 0b0,
};

/**
 * @brief The ADC comparator polarity
 */
enum ads1115_polarity {
	ADS1115_ACT_LOW  = 0b0,
	ADS1115_ACT_HIGH = 0b1,
	ADS1115_DEF_POL = 0b0,
};

/**
 * @brief The ADC comparator latch type
 */
enum ads1115_latch {
	ADS1115_NO_LATCH = 0b0,
	ADS1115_LATCH        = 0b1,
	ADS1115_DEF_LATCH = 0b0,
};

/**
 * @brief The ADC comparator comparator queue and disable
 */
enum ads1115_que {
	ADS1115_ONE_CONV         = 0b00,
	ADS1115_TWO_CONV        = 0b01,
	ADS1115_FOUR_CONV       = 0b10,
	ADS1115_COMP_DISABLE  = 0b11,
};

/**
 *  @brief The available configuration parameters
 */
struct ads1115_config {
		enum ads1115_os os;
		enum ads1115_pin pin;
		enum ads1115_gain gain;
		enum ads1115_mode mode;
		enum ads1115_data_rate data_rate;
	    enum ads1115_comparator comp;
		enum ads1115_polarity polarity;
		enum ads1115_latch latch;
		enum ads1115_que que;
};

/**
 * @brief A parsed read configuration
 */
typedef  struct ads1115_config ads1115_config_t;

/**
 * @brief starts a single shot conversion of the ADC, supporting both blocking and non-block
 * 			mode.
 * @param i2c_conf
 * @param conf
 * @return An error if something went wrong
 */
HAL_StatusTypeDef ads1115_start_reading(const ads1115_i2c_conf_t* i2c_conf,
		ads1115_config_t conf);

/**
 * @brief encodes the configuration to a raw result for the ADC config reg.
 * @param conf
 * @return
 */
ads1115_raw_conf_t ads1115_encode_cfg(const ads1115_config_t * conf);

/**
 * @brief decodes the raw config into a parsed version.
 * @param raw_conf
 * @return
 */
ads1115_config_t   ads1115_decode_cfg(const ads1115_raw_conf_t  raw_conf);

/**
 * @brief Read the raw value off of one of the ADC pins
 *
 * @param i2c_conf An open ADC i2c_conf
 * @param rd_conf The read configuration (pin, gain, data rate, etc)
 *
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_read_cfg(const ads1115_i2c_conf_t* i2c_conf,
		ads1115_config_t  * conf);

/**
 * @brief Reconfigure ADC
 *
 * @param i2c_conf An open ADC i2c_conf
 * @param rd_conf The read configuration (pin, gain, data rate, etc)
 *
 * @return A valid i2c_conf on success, error code on failure
 */
HAL_StatusTypeDef ads1115_write_cfg(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t * conf);

/**
 * @brief Reads the raw value off of one of the ADC pins and assigns a converted
 * voltage based on the gain
 *
 * @param i2c_conf An open ADC i2c_conf
 * @param conf The read configuration (pin, gain, data rate, etc)
 * @param uv
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_read_adc_millivolts(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t * conf, int32_t* uv);

/**
 * @brief Reports the low comparator threshold settings in uv
 * @param i2c_conf
 * @param uv_value
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_read_low_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value);

/**
 * @brief Reports the high comparator threshold settings in uv
 * @param i2c_conf
 * @param conf
 * @param uv_value
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_read_high_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value);

/**
 * @brief obtains the adc channel reading in microvolts
 * @param i2c_conf
 * @param conf
 * @param uv
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_read_adc_microvolts(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv);

/**
 * @brief Writes to the low comparator threshold settings in uv
 * @param i2c_conf
 * @param conf
 * @param uv_value
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_write_low_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value);

/**
 * @brief Writes to the high comparator threshold settings in uv
 *
 * @param i2c_conf
 * @param conf
 * @param uv_value
 * @return An error if something goes wrong
 */
HAL_StatusTypeDef ads1115_write_high_thresh(const ads1115_i2c_conf_t* i2c_conf,
		const ads1115_config_t *  conf, int32_t* uv_value);

#endif
