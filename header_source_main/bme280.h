#ifndef INC_BME280_H_
#define INC_BME280_H_

#include "stm32f4xx.h"
#include "stm32f4xx_ll_i2c.h"
#include "stdbool.h"
#include "stddef.h"
#include "stm32f4xx_ll_dma.h"

struct bme280_t;
typedef struct bme280_t* bme280_handle;

typedef uint32_t (*bme280_get_tick)(void);

typedef enum{
	bme280_i2c_addr0=0x76<<1,
	bme280_i2c_addr1=0x77<<1,
}bme280_i2c_address;

typedef enum{
	_bme280_ok=0,
	_bme280_fail=1,
	_bme280_timeout=2,
	_bme280_read_fail=3,
	_bme280_write_fail=4,
	_bme280_i2c_busy=5,
	_bme280_rxne_leak=6,
	_bme280_info_leak=7,
	_bme280_device_not_found=8,
	_bme280_dma_busy=9,
}bme280_return_status;

typedef union{
	uint8_t raw;
	struct{
		uint8_t spi_en: 1;
		uint8_t reserved: 1;
		uint8_t filter: 3;
		uint8_t t_sb: 3;
	}bits;
}bme280_config_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t mode: 2;
		uint8_t osrs_p: 3;
		uint8_t osrs_t: 3;
	}bits;
}bme280_ctrlmeas_t;

typedef union{
	uint8_t raw;
	struct{
		uint8_t osrs_h: 3;
		uint8_t reserved: 5;
	}bits;
}bme280_ctrlhum_t;

typedef struct{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint32_t timeout;
	bme280_get_tick get_tick;
	bme280_config_t config_t;
	bme280_ctrlmeas_t ctrlmeas_t;
	bme280_ctrlhum_t ctrlhum_t;
}bme280_user_configs;

bme280_return_status bme280_check_device(bme280_user_configs* config);
bme280_handle bme280_init(bme280_user_configs* config);
bme280_return_status bme280_reset(bme280_handle dev);
bme280_return_status bme280_configurate(bme280_handle dev, bme280_user_configs* config);
bme280_return_status bme280_read_data_poll(bme280_handle dev);
bme280_return_status bme280_get_value(bme280_handle dev);
void bme280_i2c_event_handler(I2C_TypeDef* i2c_handle);
void bme280_i2c_error_handler(I2C_TypeDef* i2c_handle);
void bme280_dma_rx_handler(void);
bme280_return_status bme280_read_data_dma(bme280_handle dev);


#endif /* INC_BME280_H_ */
