#include "bme280.h"
extern volatile uint8_t bme_data_ready;

typedef enum{
	dma_state_idle=0,
	dma_state_start_sent,
	dma_state_write_addr_sent,
	dma_state_reg_addr_sent,
	dma_state_restart_sent,
	dma_state_read_addr_sent,
	dma_state_reading,
}bme280_dma_state_t;

struct bme280_t{
	I2C_TypeDef* i2c_handle;
	uint8_t i2c_addr;
	uint32_t timeout;
	bme280_get_tick get_tick;
	bme280_config_t config_t;
	bme280_ctrlmeas_t ctrlmeas_t;
	bme280_ctrlhum_t ctrlhum_t;
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
	uint8_t raw_data[8];
	int32_t t_fine;
	float temperature;
	float pressure;
	float humidity;
	volatile bme280_dma_state_t dma_state;
};

typedef enum{
	addr_chip_id=0xD0,
	addr_reset=0xE0,
	addr_ctrlhum=0xF2,
	addr_status=0xF3,
	addr_ctrlmeas=0xF4,
	addr_config=0xF5,
	addr_press_msb=0xF7,
	addr_dig_t1=0x88,
	addr_dig_h1=0xA1,
	addr_dig_h2=0xE1,
}bme280_register_address;

#define max_instances 2
static struct bme280_t sensor_pool[max_instances];
static uint8_t next_free_index=0;
static bme280_handle active_dma_handle=NULL;

static uint8_t flag_busy(bme280_handle dev){
	return !LL_I2C_IsActiveFlag_BUSY(dev->i2c_handle);
}

static uint8_t flag_sb(bme280_handle dev){
	return LL_I2C_IsActiveFlag_SB(dev->i2c_handle);
}

static uint8_t flag_addr(bme280_handle dev){
	return LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle);
}

static uint8_t flag_txe(bme280_handle dev){
	return LL_I2C_IsActiveFlag_TXE(dev->i2c_handle);
}

static uint8_t flag_rxne(bme280_handle dev){
	return LL_I2C_IsActiveFlag_RXNE(dev->i2c_handle);
}

static uint8_t flag_btf(bme280_handle dev){
	return LL_I2C_IsActiveFlag_BTF(dev->i2c_handle);
}

static uint8_t flag_addr_or_af(bme280_handle dev){
	return LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle) || LL_I2C_IsActiveFlag_AF(dev->i2c_handle);
}

typedef uint8_t (*bme280_condition_t)(bme280_handle dev);
static bme280_return_status wait_for_condition(bme280_handle dev, bme280_condition_t condition_met){
	uint32_t time = dev->get_tick();
	while(condition_met(dev) == 0){
		if(dev->get_tick() - time > dev->timeout) return _bme280_timeout;
	}
	return _bme280_ok;
}

static bme280_return_status read_register(bme280_handle dev, uint8_t reg_addr, uint8_t* rxdata, uint8_t size){
	if(size == 0) return _bme280_fail;
	if(LL_I2C_IsActiveFlag_RXNE(dev->i2c_handle)){
		 (void)LL_I2C_ReceiveData8(dev->i2c_handle); return _bme280_rxne_leak;
	}
	if(wait_for_condition(dev, flag_busy)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_i2c_busy;
	}
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	if(wait_for_condition(dev, flag_sb)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr);
	if(wait_for_condition(dev, flag_addr)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_TransmitData8(dev->i2c_handle, reg_addr);
	if(wait_for_condition(dev, flag_txe)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	if(wait_for_condition(dev, flag_sb)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr|1);
	if(wait_for_condition(dev, flag_addr)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_ACK);
	while(size>1){
		if(wait_for_condition(dev, flag_rxne)!=_bme280_ok){
			LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
		}
		*rxdata=LL_I2C_ReceiveData8(dev->i2c_handle);
		rxdata++;
		size--;
	}
	LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_NACK);
	if(wait_for_condition(dev, flag_rxne)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	*rxdata=LL_I2C_ReceiveData8(dev->i2c_handle);
	LL_I2C_GenerateStopCondition(dev->i2c_handle);
	return _bme280_ok;
}

static bme280_return_status write_register(bme280_handle dev, uint8_t reg_addr, uint8_t* txdata, uint8_t size){
	if(size==0) return _bme280_fail;
	if(LL_I2C_IsActiveFlag_RXNE(dev->i2c_handle)){
		 (void)LL_I2C_ReceiveData8(dev->i2c_handle); return _bme280_rxne_leak;
	}
	if(wait_for_condition(dev, flag_busy)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_i2c_busy;
	}
	LL_I2C_GenerateStartCondition(dev->i2c_handle);
	if(wait_for_condition(dev, flag_sb)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr);
	if(wait_for_condition(dev, flag_addr)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
	LL_I2C_TransmitData8(dev->i2c_handle, reg_addr);
	if(wait_for_condition(dev, flag_txe)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	while(size>0){
		LL_I2C_TransmitData8(dev->i2c_handle, *txdata);
		txdata++;
		size--;
		if(wait_for_condition(dev, flag_txe)!=_bme280_ok){
			LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
		}
	}
	if(wait_for_condition(dev, flag_btf)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_GenerateStopCondition(dev->i2c_handle);
	return _bme280_ok;
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

bme280_return_status bme280_check_device(bme280_user_configs* config){
	if(config==NULL) return _bme280_info_leak;
	if(config->get_tick==NULL) return _bme280_info_leak;
	if(config->i2c_addr != bme280_i2c_addr0 && config->i2c_addr != bme280_i2c_addr1) return _bme280_info_leak;
	if(config->i2c_handle != I2C1 && config->i2c_handle != I2C2 && config->i2c_handle != I2C3) return _bme280_info_leak;
	if(config->timeout==0) return _bme280_info_leak;
	if(LL_I2C_IsActiveFlag_RXNE(config->i2c_handle)){
		(void)LL_I2C_ReceiveData8(config->i2c_handle); LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_rxne_leak;
	}
	if(LL_I2C_IsActiveFlag_BUSY(config->i2c_handle)){
		LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_i2c_busy;
	}
	LL_I2C_GenerateStartCondition(config->i2c_handle);
	if(wait_for_condition((bme280_handle)config, flag_sb)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_timeout;
	}
	LL_I2C_TransmitData8(config->i2c_handle, config->i2c_addr);
	if(wait_for_condition((bme280_handle)config, flag_addr_or_af)==_bme280_timeout){
		LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_timeout;
	}
	if(LL_I2C_IsActiveFlag_ADDR(config->i2c_handle)){
		LL_I2C_ClearFlag_ADDR(config->i2c_handle); LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_ok;
	}
	LL_I2C_ClearFlag_AF(config->i2c_handle); LL_I2C_GenerateStopCondition(config->i2c_handle); return _bme280_device_not_found;
}

bme280_handle bme280_init(bme280_user_configs* config){
	if(next_free_index >= max_instances) return NULL;
	if(config==NULL) return NULL;
	if(config->get_tick==NULL) return NULL;
	if(config->i2c_addr != bme280_i2c_addr0 && config->i2c_addr != bme280_i2c_addr1) return NULL;
	if(config->i2c_handle != I2C1 && config->i2c_handle != I2C2 && config->i2c_handle != I2C3) return NULL;
	if(config->timeout==0) return NULL;
	bme280_handle dev = &sensor_pool[next_free_index];
	next_free_index++;
	dev->i2c_handle=config->i2c_handle;
	dev->i2c_addr=config->i2c_addr;
	dev->get_tick=config->get_tick;
	dev->timeout=config->timeout;
	dev->config_t.raw=config->config_t.raw;
	dev->ctrlhum_t.raw=config->ctrlhum_t.raw;
	dev->ctrlmeas_t.raw=config->ctrlmeas_t.raw;
	uint8_t buffer;
	bme280_return_status status = read_register(dev, addr_chip_id, &buffer, 1);
	if(status!=_bme280_ok || buffer!=0x60){
		next_free_index--;
		return NULL;
	}
	uint8_t comp_buffer[24];
	status = read_register(dev, addr_dig_t1, comp_buffer, 24);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	dev->dig_T1=(uint16_t)(comp_buffer[1]<<8)|(comp_buffer[0]);
	dev->dig_T2=(int16_t)(comp_buffer[3]<<8)|(comp_buffer[2]);
	dev->dig_T3=(int16_t)(comp_buffer[5]<<8)|(comp_buffer[4]);
	dev->dig_P1=(uint16_t)(comp_buffer[7]<<8)|(comp_buffer[6]);
	dev->dig_P2=(int16_t)(comp_buffer[9]<<8)|(comp_buffer[8]);
	dev->dig_P3=(int16_t)(comp_buffer[11]<<8)|(comp_buffer[10]);
	dev->dig_P4=(int16_t)(comp_buffer[13]<<8)|(comp_buffer[12]);
	dev->dig_P5=(int16_t)(comp_buffer[15]<<8)|(comp_buffer[14]);
	dev->dig_P6=(int16_t)(comp_buffer[17]<<8)|(comp_buffer[16]);
	dev->dig_P7=(int16_t)(comp_buffer[19]<<8)|(comp_buffer[18]);
	dev->dig_P8=(int16_t)(comp_buffer[21]<<8)|(comp_buffer[20]);
	dev->dig_P9=(int16_t)(comp_buffer[23]<<8)|(comp_buffer[22]);
	status = read_register(dev, addr_dig_h1, &(dev->dig_H1), 1);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	status = read_register(dev, addr_dig_h2, comp_buffer, 7);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	dev->dig_H2=(int16_t)(comp_buffer[1]<<8)|(comp_buffer[0]);
	dev->dig_H3=comp_buffer[2];
	dev->dig_H4=(int16_t)((comp_buffer[3]<<4)|(comp_buffer[4]&0x0F));
	dev->dig_H5=(int16_t)((comp_buffer[5]<<4)|(comp_buffer[4]>>4));
	dev->dig_H6=(int8_t)comp_buffer[6];
	dev->dma_state=dma_state_idle; //dma state işte burada alo
	status = write_register(dev, addr_ctrlhum, &(dev->ctrlhum_t.raw), 1);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	status = write_register(dev, addr_config, &(dev->config_t.raw), 1);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	status = write_register(dev, addr_ctrlmeas, &(dev->ctrlmeas_t.raw), 1);
	if(status!=_bme280_ok){
		next_free_index--;
		return NULL;
	}
	return dev;
}

bme280_return_status bme280_reset(bme280_handle dev){
	uint8_t buffer = 0xB6;
	bme280_return_status status = write_register(dev, addr_reset, &buffer, 1);
	if(status!=_bme280_ok) return status;
	status = write_register(dev, addr_ctrlhum, &(dev->ctrlhum_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	status = write_register(dev, addr_config, &(dev->config_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	status = write_register(dev, addr_ctrlmeas, &(dev->ctrlmeas_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	return _bme280_ok;
}

bme280_return_status bme280_configurate(bme280_handle dev, bme280_user_configs* config){
	if(config==NULL) return _bme280_info_leak;
	bme280_return_status status;
	dev->ctrlhum_t.raw = config->ctrlhum_t.raw;
	dev->config_t.raw = config->config_t.raw;
	dev->ctrlmeas_t.raw = config->ctrlmeas_t.raw;
	uint8_t sleep_mode=0;
	status = write_register(dev, addr_ctrlmeas, &sleep_mode, 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	status = write_register(dev, addr_ctrlhum, &(dev->ctrlhum_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	status = write_register(dev, addr_config, &(dev->config_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	status = write_register(dev, addr_ctrlmeas, &(dev->ctrlmeas_t.raw), 1);
	if(status!=_bme280_ok) return _bme280_write_fail;
	return _bme280_ok;
}

bme280_return_status bme280_read_data_poll(bme280_handle dev){
	if(dev==NULL) return _bme280_info_leak;
	return read_register(dev, addr_press_msb, dev->raw_data, 8);
}

bme280_return_status bme280_get_value(bme280_handle dev){
	if(dev==NULL) return _bme280_info_leak;
	int32_t raw_pressure = (int32_t)((uint32_t)dev->raw_data[0]<<12)|((uint32_t)dev->raw_data[1]<<4)|((uint32_t)dev->raw_data[2]>>4);
	int32_t raw_temperature = (int32_t)((uint32_t)dev->raw_data[3]<<12)|((uint32_t)dev->raw_data[4]<<4)|((uint32_t)dev->raw_data[5]>>4);
	int32_t raw_humidity = (int32_t)((uint32_t)dev->raw_data[6]<<8)|(dev->raw_data[7]);
	dev->temperature=(float)compensate_temperature(dev, raw_temperature)/100.0f;
	dev->pressure=(float)compensate_pressure(dev, raw_pressure)/25600.0f;
	dev->humidity=(float)compensate_humidity(dev, raw_humidity)/1024.0f;
	return _bme280_ok;
}

static void bme280_dma_cleanup(bme280_handle dev){
	if(dev==NULL) return;
	LL_I2C_DisableIT_EVT(dev->i2c_handle);
	LL_I2C_DisableIT_BUF(dev->i2c_handle);
	LL_I2C_DisableIT_ERR(dev->i2c_handle);
	LL_I2C_DisableDMAReq_RX(dev->i2c_handle);
	LL_I2C_DisableLastDMA(dev->i2c_handle);
	LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_0);
	LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
	dev->dma_state=dma_state_idle;
	active_dma_handle=NULL;
}

bme280_return_status bme280_read_data_dma(bme280_handle dev){
	if(dev==NULL) return _bme280_info_leak;
	if(active_dma_handle!=NULL) return _bme280_dma_busy;
	if(dev->dma_state!=dma_state_idle) return _bme280_dma_busy;
	if(wait_for_condition(dev, flag_busy)!=_bme280_ok){
		LL_I2C_GenerateStopCondition(dev->i2c_handle); return _bme280_i2c_busy;
	}
	active_dma_handle=dev;
	dev->dma_state=dma_state_start_sent;

    LL_I2C_ClearFlag_AF(dev->i2c_handle);
    LL_I2C_ClearFlag_BERR(dev->i2c_handle);
    LL_DMA_ClearFlag_TC0(DMA1);
    LL_DMA_ClearFlag_TE0(DMA1);

    LL_I2C_EnableIT_EVT(dev->i2c_handle);
    LL_I2C_EnableIT_BUF(dev->i2c_handle);
    LL_I2C_EnableIT_ERR(dev->i2c_handle);

    LL_DMA_EnableIT_TE(DMA1, LL_DMA_STREAM_0);
    LL_DMA_EnableIT_TC(DMA1, LL_DMA_STREAM_0);

    LL_I2C_GenerateStartCondition(dev->i2c_handle);
    return _bme280_ok;
}

void bme280_i2c_error_handler(I2C_TypeDef* i2c_handle){
	bme280_handle dev = active_dma_handle;
	if(dev==NULL || dev->i2c_handle!=i2c_handle) return;
	if (LL_I2C_IsActiveFlag_AF(dev->i2c_handle)) { LL_I2C_ClearFlag_AF(dev->i2c_handle); }
	if (LL_I2C_IsActiveFlag_BERR(dev->i2c_handle)) { LL_I2C_ClearFlag_BERR(dev->i2c_handle); }
	if (LL_I2C_IsActiveFlag_OVR(dev->i2c_handle)) { LL_I2C_ClearFlag_OVR(dev->i2c_handle); }
	if(LL_I2C_IsActiveFlag_ARLO(dev->i2c_handle)) { LL_I2C_ClearFlag_ARLO(dev->i2c_handle); }
	bme280_dma_cleanup(dev);
}

void bme280_dma_rx_handler(void){
	bme280_handle dev = active_dma_handle;
	if(dev==NULL) return;
	if(LL_DMA_IsActiveFlag_TE0(DMA1)){
		LL_DMA_ClearFlag_TE0(DMA1); bme280_dma_cleanup(dev);
	}
	if(LL_DMA_IsActiveFlag_TC0(DMA1)){
		LL_DMA_ClearFlag_TC0(DMA1); LL_I2C_GenerateStopCondition(dev->i2c_handle); bme280_dma_cleanup(dev); bme280_get_value(dev); bme_data_ready=1;
	}
}

void bme280_i2c_event_handler(I2C_TypeDef* i2c_handle){
	bme280_handle dev = active_dma_handle;
	if(dev==NULL || dev->i2c_handle != i2c_handle) return;
	switch(dev->dma_state){
	case dma_state_start_sent:{
		if(LL_I2C_IsActiveFlag_SB(dev->i2c_handle)){
			LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr);
			dev->dma_state=dma_state_write_addr_sent;
		}
		break;
	}
	case dma_state_write_addr_sent:{
		if(LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle)){
			LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
			LL_I2C_TransmitData8(dev->i2c_handle, addr_press_msb);
			dev->dma_state=dma_state_reg_addr_sent;
		}
		break;
	}
	case dma_state_reg_addr_sent:{
		if(LL_I2C_IsActiveFlag_TXE(dev->i2c_handle)){
			LL_I2C_GenerateStartCondition(dev->i2c_handle);
			dev->dma_state=dma_state_restart_sent;
		}
		break;
	}
	case dma_state_restart_sent:{
		if(LL_I2C_IsActiveFlag_SB(dev->i2c_handle)){
			LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr|1);
			dev->dma_state=dma_state_read_addr_sent;
		}
		break;
	}
	case dma_state_read_addr_sent:{
		if(LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle)){
			LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
			LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_I2C_DMA_GetRegAddr(dev->i2c_handle));
			LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)dev->raw_data);
			LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 8);
			LL_I2C_EnableLastDMA(dev->i2c_handle);
			LL_I2C_EnableDMAReq_RX(dev->i2c_handle);
			LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);
			LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_ACK);
			LL_I2C_DisableIT_EVT(dev->i2c_handle);
			LL_I2C_DisableIT_BUF(dev->i2c_handle);
			dev->dma_state=dma_state_reading;
			LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
		}
		break;
	}
	case dma_state_reading: break;
	case dma_state_idle: break;
	default: break;
	}
}
