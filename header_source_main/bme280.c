/*
 * bme280.c
 *
 *  Created on: Nov 4, 2025
 *      Author: Enes
 */

#include "bme280.h"

typedef enum{
	chip_id = 0xD0,
	dig_T1 = 0x88,
	dig_H1 = 0xA1,
	dig_H2 = 0xE1,
	addr_config = 0xF5,
	addr_ctrlmeas = 0xF4,
	addr_ctrlhum = 0xF2,
	press_msb = 0xF7
}bme280_register_address;

struct bme280_t{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint32_t timeout;
	bme280_get_tick_func_t get_tick;
	bme280_config_reg config;
	bme280_ctrlmeas_reg ctrlmeas;
	bme280_ctrlhum_reg ctrlhum;
	uint16_t dig_T1;
	int16_t dig_T2;
	int16_t dig_T3;
	uint16_t dig_P1;
	int16_t dig_P2;
	int16_t dig_P3;
	int16_t dig_P4;
	int16_t dig_P5;
	int16_t dig_P6;
	int16_t dig_P7;
	int16_t dig_P8;
	int16_t dig_P9;
	uint8_t dig_H1;
	int16_t dig_H2;
	uint8_t dig_H3;
	int16_t dig_H4;
	int16_t dig_H5;
	int8_t dig_H6;
	int32_t t_fine;
	uint8_t raw_data[8];
	float temperature;
	float pressure;
	float humidity;
};

#define BME280_MAX_INSTANCES   2
static struct bme280_t g_sensor_pool[BME280_MAX_INSTANCES];
static uint8_t g_next_free_index=0;

bme280_return_stats static read_register(bme280_handle dev, uint8_t reg_addr, uint8_t* rxdata, uint8_t size){
	uint32_t time;
	if(size == 0) return bme280_read_fail;
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_SB(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_TransmitData8(dev->i2c_handle, reg_addr);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_TXE(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_SB(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr | 1);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_ACK);
	while(size-1){
		time = dev->get_tick();
		while(!LL_I2C_IsActiveFlag_RXNE(dev->i2c_handle)){
			if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
		}
		*rxdata = LL_I2C_ReceiveData8(dev->i2c_handle);
		rxdata++;
		size--;
	}
	LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_NACK);
	LL_I2C_GenerateStopCondition(dev->i2c_handle);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_RXNE(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	*rxdata = LL_I2C_ReceiveData8(dev->i2c_handle);
	return bme280_ok;
}

bme280_return_stats static write_register(bme280_handle dev, uint8_t reg_addr, uint8_t* txdata, uint8_t size){
	uint32_t time = 0;
	if(size == 0) return bme280_write_fail;
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_SB(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_TransmitData8(dev->i2c_handle, reg_addr);
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_TXE(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	while(size > 0){
		LL_I2C_TransmitData8(dev->i2c_handle, *txdata);
		time = dev->get_tick();
		while(!LL_I2C_IsActiveFlag_TXE(dev->i2c_handle)){
			if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
		}
		txdata++;
		size--;
	}
	time = dev->get_tick();
	while(!LL_I2C_IsActiveFlag_BTF(dev->i2c_handle)){
		if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
	}
	LL_I2C_GenerateStopCondition(dev->i2c_handle);
	return bme280_ok;
}

bme280_return_stats static find_bme280(bme280_user_configs* config){
	uint32_t time;
	LL_I2C_GenerateStartCondition(config->i2c_handle);
	time = config->get_tick();
	while(!LL_I2C_IsActiveFlag_SB(config->i2c_handle)){
		if(config->get_tick() - time > config->timeout) return bme280_timeout;
	}
	LL_I2C_TransmitData8(config->i2c_handle, config->i2c_addr);
	time = config->get_tick();
	while(!LL_I2C_IsActiveFlag_ADDR(config->i2c_handle) && !LL_I2C_IsActiveFlag_AF(config->i2c_handle)){
		if(config->get_tick() - time > config->timeout){
			LL_I2C_GenerateStopCondition(config->i2c_handle);
			return bme280_timeout;
		}
	}
	if(!LL_I2C_IsActiveFlag_ADDR(config->i2c_handle)){
		LL_I2C_ClearFlag_AF(config->i2c_handle);
		LL_I2C_GenerateStopCondition(config->i2c_handle);
		return bme280_device_not_found;
	}
	LL_I2C_ClearFlag_ADDR(config->i2c_handle);
	LL_I2C_GenerateStopCondition(config->i2c_handle);
	return bme280_ok;
}

bme280_handle bme280_init(bme280_user_configs* config){
	if(g_next_free_index >= BME280_MAX_INSTANCES) return NULL;
	if(config->get_tick == NULL) return NULL;
	if((config->i2c_addr != i2c_addr_0) && (config->i2c_addr != i2c_addr_1)) return NULL;
	if(config->i2c_handle != I2C1 && config->i2c_handle != I2C2 && config->i2c_handle != I2C3) return NULL;
	uint32_t time = config->get_tick();
	while(LL_I2C_IsActiveFlag_BUSY(config->i2c_handle)) if(config->get_tick() - time > config->timeout) return NULL;
	if(find_bme280(config)!=bme280_ok) return NULL;
	bme280_handle dev = &g_sensor_pool[g_next_free_index];
	g_next_free_index++;
	dev->i2c_handle = config->i2c_handle;
	dev->i2c_addr = config->i2c_addr;
	dev->timeout = config->timeout;
	dev->get_tick = config->get_tick;
	uint8_t buffer;
	read_register(dev, chip_id, &buffer, 1);
	if(buffer!=0x60) return NULL;
	uint8_t temp_buffer[26];
	bme280_return_stats status;
	status = read_register(dev, dig_T1, temp_buffer, 24);
	if(status != bme280_ok) return NULL;
	dev->dig_T1 = (uint16_t)((temp_buffer[1] << 8)|(temp_buffer[0]));
	dev->dig_T2 = (int16_t)((temp_buffer[3] << 8)|(temp_buffer[2]));
	dev->dig_T3 = (int16_t)((temp_buffer[5] << 8)|(temp_buffer[4]));
	dev->dig_P1 = (uint16_t)((temp_buffer[7] << 8)|(temp_buffer[6]));
	dev->dig_P2 = (int16_t)((temp_buffer[9] << 8)|(temp_buffer[8]));
	dev->dig_P3 = (int16_t)((temp_buffer[11] << 8)|(temp_buffer[10]));
	dev->dig_P4 = (int16_t)((temp_buffer[13] << 8)|(temp_buffer[12]));
	dev->dig_P5 = (int16_t)((temp_buffer[15] << 8)|(temp_buffer[14]));
	dev->dig_P6 = (int16_t)((temp_buffer[17] << 8)|(temp_buffer[16]));
	dev->dig_P7 = (int16_t)((temp_buffer[19] << 8)|(temp_buffer[18]));
	dev->dig_P8 = (int16_t)((temp_buffer[21] << 8)|(temp_buffer[20]));
	dev->dig_P9 = (int16_t)((temp_buffer[23] << 8)|(temp_buffer[22]));
	status = read_register(dev, dig_H1, &(dev->dig_H1), 1);
	if(status != bme280_ok) return NULL;
	status = read_register(dev, dig_H2, temp_buffer, 7);
	if(status != bme280_ok) return NULL;
	dev->dig_H2 = (int16_t)((temp_buffer[1] << 8)|(temp_buffer[0]));
	dev->dig_H3 = temp_buffer[2];
	dev->dig_H4 = (int16_t)((temp_buffer[3] << 4)|(temp_buffer[4] & 0x0F));
	dev->dig_H5 = (int16_t)((temp_buffer[5] << 4)|(temp_buffer[4] >> 4));
	dev->dig_H6 = (int8_t)(temp_buffer[6]);
	status = bme280_configurate(dev, config);
	if(status!=bme280_ok) return NULL;
	return dev;
}

bme280_return_stats bme280_configurate(bme280_handle dev, bme280_user_configs* config){
	dev->config.raw = config->config.raw;
	dev->ctrlhum.raw = config->ctrlhum.raw;
	dev->ctrlmeas.raw = config->ctrlmeas.raw;
	bme280_return_stats status;
	status = write_register(dev, addr_ctrlhum, &(dev->ctrlhum.raw), 1);
	if(status!=bme280_ok) return bme280_fail;
	status = write_register(dev, addr_config, &(dev->config.raw), 1);
	if(status!=bme280_ok) return bme280_fail;
	status = write_register(dev, addr_ctrlmeas, &(dev->ctrlmeas.raw), 1);
	if(status!=bme280_ok) return bme280_fail;
	return bme280_ok;
}

static int32_t inline compensate_temperature(bme280_handle dev, int32_t adc_T)
{
    int32_t var1, var2, T;
    var1 = ((((adc_T >> 3) - ((int32_t)dev->dig_T1 << 1))) * ((int32_t)dev->dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((int32_t)dev->dig_T1)) * ((adc_T >> 4) - ((int32_t)dev->dig_T1))) >> 12) * ((int32_t)dev->dig_T3)) >> 14;
    dev->t_fine = var1 + var2;
    T = (dev->t_fine * 5 + 128) >> 8;
    return T;
}

static uint32_t inline compensate_pressure(bme280_handle dev, int32_t adc_P)
{
    int64_t var1, var2, p;
    var1 = ((int64_t)dev->t_fine) - 128000;
    var2 = var1 * var1 * (int64_t)dev->dig_P6;
    var2 = var2 + ((var1 * (int64_t)dev->dig_P5) << 17);
    var2 = var2 + (((int64_t)dev->dig_P4) << 35);
    var1 = ((var1 * var1 * (int64_t)dev->dig_P3) >> 8) + ((var1 * (int64_t)dev->dig_P2) << 12);
    var1 = (((((int64_t)1) << 47) + var1)) * ((int64_t)dev->dig_P1) >> 33;
    if (var1 == 0) { return 0; }
    p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = (((int64_t)dev->dig_P9) * (p >> 13) * (p >> 13)) >> 25;
    var2 = (((int64_t)dev->dig_P8) * p) >> 19;
    p = ((p + var1 + var2) >> 8) + (((int64_t)dev->dig_P7) << 4);
    return (uint32_t)p;
}

static uint32_t inline compensate_humidity(bme280_handle dev, int32_t adc_H)
{
    int32_t v_x1_u32r;
    v_x1_u32r = (dev->t_fine - ((int32_t)76800));
    v_x1_u32r = (((((adc_H << 14) - ((((int32_t)dev->dig_H4) << 20) + (((int32_t)dev->dig_H5) * v_x1_u32r))) +
                 ((int32_t)16384)) >> 15) * (((((((v_x1_u32r * ((int32_t)dev->dig_H6)) >> 10) *
                 (((v_x1_u32r * ((int32_t)dev->dig_H3)) >> 11) + ((int32_t)32768))) >> 10) +
                 ((int32_t)2097152)) * ((int32_t)dev->dig_H2) + 8192) >> 14));
    v_x1_u32r = (v_x1_u32r - (((((v_x1_u32r >> 15) * (v_x1_u32r >> 15)) >> 7) * ((int32_t)dev->dig_H1)) >> 4));
    v_x1_u32r = (v_x1_u32r < 0 ? 0 : v_x1_u32r);
    v_x1_u32r = (v_x1_u32r > 419430400 ? 419430400 : v_x1_u32r);
    return (uint32_t)(v_x1_u32r >> 12);
}

bme280_return_stats bme280_read_data_poll(bme280_handle dev){
	bme280_return_stats status;
	status = read_register(dev, press_msb, dev->raw_data, 8);
	return status;
}

bme280_return_stats  bme280_get_values(bme280_handle dev){
    int32_t adc_P = (int32_t)(((uint32_t)dev->raw_data[0] << 12) | ((uint32_t)dev->raw_data[1] << 4) | ((uint32_t)dev->raw_data[2] >> 4));
    int32_t adc_T = (int32_t)(((uint32_t)dev->raw_data[3] << 12) | ((uint32_t)dev->raw_data[4] << 4) | ((uint32_t)dev->raw_data[5] >> 4));
    int32_t adc_H = (int32_t)(((uint32_t)dev->raw_data[6] << 8) | dev->raw_data[7]);

    dev->temperature = (float)compensate_temperature(dev, adc_T) / 100.0f;
    dev->pressure = (float)compensate_pressure(dev, adc_P) / 256.0f / 100.0f;
    dev->humidity = (float)compensate_humidity(dev, adc_H) / 1024.0f;
    return bme280_ok;
}

bme280_return_stats bme280_get_data_forced(bme280_handle dev)
{
    bme280_return_stats status;
    uint32_t time;
    uint8_t status_reg;
    dev->ctrlmeas.bits.mode = 0b01;
    status = write_register(dev, addr_ctrlmeas, &(dev->ctrlmeas.raw), 1);
    if(status != bme280_ok) return status;
    time = dev->get_tick();
    do
    {
        status = read_register(dev, 0xF3, &status_reg, 1);
        if (status != bme280_ok) return status;
        if(dev->get_tick() - time > dev->timeout) return bme280_timeout;
    } while (status_reg & (1 << 3));
    status = bme280_read_data_poll(dev);
    if (status != bme280_ok) return status;
    bme280_get_values(dev);
    return bme280_ok;
}
