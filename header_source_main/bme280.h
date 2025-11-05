/*
 * bme280.h
 *
 *  Created on: Nov 4, 2025
 *      Author: Enes
 */

#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "stm32f407xx.h"
#include "stm32f4xx_ll_i2c.h"
#include "stddef.h"
#include "stdbool.h"

struct bme280_t;
typedef struct bme280_t* bme280_handle;

typedef uint32_t (*bme280_get_tick_func_t)(void);

typedef enum{
	i2c_addr_0= 0x76 << 1,
	i2c_addr_1= 0x77 << 1,
}bme280_i2c_address;

typedef enum{
	bme280_ok=0,
	bme280_fail=1,
	bme280_timeout=2,
	bme280_read_fail=3,
	bme280_write_fail=4,
	bme280_device_not_found=5,
}bme280_return_stats;

typedef union{
	uint8_t raw;
	struct{
		uint8_t spi_en: 1;
		uint8_t reserved: 1;
		uint8_t filter: 3;
		uint8_t t_sb: 3;
	}bits;
}bme280_config_reg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t mode: 2;
		uint8_t osrs_p: 3;
		uint8_t osrs_t: 3;
	}bits;
}bme280_ctrlmeas_reg;

typedef union{
	uint8_t raw;
	struct{
		uint8_t osrs_h: 3;
		uint8_t reserved: 5;
	}bits;
}bme280_ctrlhum_reg;

typedef struct{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint32_t timeout;
	bme280_config_reg config;
	bme280_ctrlmeas_reg ctrlmeas;
	bme280_ctrlhum_reg ctrlhum;
	bme280_get_tick_func_t get_tick;
}bme280_user_configs;

bme280_handle bme280_init(bme280_user_configs* config);
bme280_return_stats bme280_configurate(bme280_handle dev, bme280_user_configs* config);
bme280_return_stats bme280_read_data_poll(bme280_handle dev);
bme280_return_stats  bme280_get_values(bme280_handle dev);
bme280_return_stats bme280_get_data_forced(bme280_handle dev);

#endif /* INC_BME280_H_ */
