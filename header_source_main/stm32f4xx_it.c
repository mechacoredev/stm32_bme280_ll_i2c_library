/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32f4xx_it.c
  * @brief   Interrupt Service Routines.
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "bme280.h"
/* bme280_get_values'i buradan çağırabilmek için prototip */
bme280_return_stats bme280_get_values(bme280_handle dev);
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */
typedef enum {
    BME280_DMA_STATE_IDLE = 0,
    BME280_DMA_STATE_START,
    BME280_DMA_STATE_WRITE_ADDR_SENT,
    BME280_DMA_STATE_REG_ADDR_SENT,
    BME280_DMA_STATE_RESTART_SENT,
    BME280_DMA_STATE_READ_ADDR_SENT,
    BME280_DMA_STATE_READING
} bme280_dma_state_t;

#define BME280_PRESS_MSB_REG 0xF7
/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
extern volatile uint32_t uwtick;
extern bme280_handle my_sensor;
extern uint8_t success;
extern bme280_handle g_bme280_dma_handle;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */
/* Hata veya başarı durumunda tüm sistemi temizleyen yardımcı fonksiyon */
void BME280_DMA_Cleanup()
{
    if (g_bme280_dma_handle == NULL) return;

    LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
    LL_DMA_DisableIT_TC(DMA1, LL_DMA_STREAM_0);
    LL_DMA_DisableIT_TE(DMA1, LL_DMA_STREAM_0);

    LL_I2C_DisableDMAReq_RX(I2C1);
    LL_I2C_DisableLastDMA(I2C1); // Bunu da temizle
    LL_I2C_DisableIT_EVT(I2C1);
    LL_I2C_DisableIT_BUF(I2C1);
    LL_I2C_DisableIT_ERR(I2C1);

    /* * LL_I2C_GenerateStopCondition(I2C1);
     * BU SATIRI BURADAN SİLİYORUZ!
     * Artık STOP, DMA_IRQHandler içinde gönderilecek.
     */

    g_bme280_dma_handle->i2c_dma_state = BME280_DMA_STATE_IDLE;
    g_bme280_dma_handle = NULL;
}
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/* Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/* ... (NMI_Handler -> SysTick_Handler arası aynı) ... */
void NMI_Handler(void) { while (1) {} }
void HardFault_Handler(void) { while (1) {} }
void MemManage_Handler(void) { while (1) {} }
void BusFault_Handler(void) { while (1) {} }
void UsageFault_Handler(void) { while (1) {} }
void SVC_Handler(void) {}
void DebugMon_Handler(void) {}
void PendSV_Handler(void) {}

void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */
	uwtick++;
  /* USER CODE END SysTick_IRQn 0 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/******************************************************************************/

/**
  * @brief This function handles DMA1 stream0 global interrupt.
  */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */
    bme280_handle dev = g_bme280_dma_handle;
    if (dev == NULL) {
        LL_DMA_ClearFlag_TC0(DMA1);
        LL_DMA_ClearFlag_TE0(DMA1);
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
        return;
    }

    // BAŞARI: Transfer Tamamlandı
    if (LL_DMA_IsActiveFlag_TC0(DMA1))
    {
        LL_DMA_ClearFlag_TC0(DMA1);

        /* * ===================================================================
         * ===                    ÇÖZÜM: MANUEL STOP                       ===
         * ===================================================================
         * 'LAST' biti NACK yolladı, DMA bitti. Şimdi hattı
         * serbest bırakmak için MANUEL olarak STOP gönderiyoruz.
         * Donanımın 'STOPF' bayrağına GÜVENMİYORUZ.
         */
        LL_I2C_GenerateStopCondition(I2C1);
        /* ----------------------------------------------------------------- */

        BME280_DMA_Cleanup(); // Tüm sistemi temizle

        bme280_get_values(dev); // Veriyi işle
        success = 1; // main.c'deki döngü için
    }

    // HATA: Transfer Hatası
    if (LL_DMA_IsActiveFlag_TE0(DMA1))
    {
        LL_DMA_ClearFlag_TE0(DMA1);
        BME280_DMA_Cleanup(); // Hata durumunda sistemi temizle
        success = 0; // Başarısız oldu
    }

  /* USER CODE END DMA1_Stream0_IRQn 0 */
}

/**
  * @brief This function handles I2C1 event interrupt.
  */
void I2C1_EV_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_EV_IRQn 0 */
    bme280_handle dev = g_bme280_dma_handle;

    if (dev == NULL) {
        LL_I2C_DisableIT_EVT(I2C1);
        LL_I2C_DisableIT_BUF(I2C1);
        return;
    }

    uint8_t state = dev->i2c_dma_state;

    // 1. Durum: (SB bayrağı) START çekildi
    if (state == BME280_DMA_STATE_START && LL_I2C_IsActiveFlag_SB(dev->i2c_handle))
    {
        LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr & ~0x01); // Write adresi
        dev->i2c_dma_state = BME280_DMA_STATE_WRITE_ADDR_SENT;
    }
    // 2. Durum: (ADDR bayrağı) Cihaz Adresi (Write) alındı
    else if (state == BME280_DMA_STATE_WRITE_ADDR_SENT && LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle))
    {
        LL_I2C_ClearFlag_ADDR(dev->i2c_handle);
        LL_I2C_TransmitData8(dev->i2c_handle, BME280_PRESS_MSB_REG); // 0xF7
        dev->i2c_dma_state = BME280_DMA_STATE_REG_ADDR_SENT;
    }
    // 3. Durum: (TXE bayrağı) Register Adresi yollandı
    else if (state == BME280_DMA_STATE_REG_ADDR_SENT && LL_I2C_IsActiveFlag_TXE(dev->i2c_handle))
    {
        LL_I2C_GenerateStartCondition(dev->i2c_handle);
        dev->i2c_dma_state = BME280_DMA_STATE_RESTART_SENT;
    }
    // 4. Durum: (SB bayrağı) Repeated-START çekildi
    else if (state == BME280_DMA_STATE_RESTART_SENT && LL_I2C_IsActiveFlag_SB(dev->i2c_handle))
    {
        LL_I2C_TransmitData8(dev->i2c_handle, dev->i2c_addr | 0x01); // Read adresi
        dev->i2c_dma_state = BME280_DMA_STATE_READ_ADDR_SENT;
    }
    // 5. Durum: (ADDR bayrağı) Cihaz Adresi (Read) alındı, DMA'yı başlat!
    else if (state == BME280_DMA_STATE_READ_ADDR_SENT && LL_I2C_IsActiveFlag_ADDR(dev->i2c_handle))
    {
        // 1. DMA konfigürasyonu (Stream 0, 8 byte)
        LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_0);
        LL_DMA_SetPeriphAddress(DMA1, LL_DMA_STREAM_0, LL_I2C_DMA_GetRegAddr(dev->i2c_handle));
        LL_DMA_SetMemoryAddress(DMA1, LL_DMA_STREAM_0, (uint32_t)dev->raw_data);
        LL_DMA_SetDataLength(DMA1, LL_DMA_STREAM_0, 8);

        // 2. I2C'ye 8 byte sonra NACK yollamasını söyle
        LL_I2C_EnableLastDMA(dev->i2c_handle);

        // 3. I2C'nin DMA isteğini aç
        LL_I2C_EnableDMAReq_RX(dev->i2c_handle);

        // 4. DMA'yı başlat
        LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_0);

        // 5. STM32F4 Hatası: Multi-byte DMA okuması için ACK'ı aç
        LL_I2C_AcknowledgeNextData(dev->i2c_handle, LL_I2C_ACK);

        // 6. I2C transferini başlatmak için ADDR bayrağını temizle!
        LL_I2C_ClearFlag_ADDR(dev->i2c_handle);

        // 7. Olay ve Buffer kesmelerinin işi bitti, artık görev DMA_IRQHandler'da
        LL_I2C_DisableIT_EVT(dev->i2c_handle);
        LL_I2C_DisableIT_BUF(dev->i2c_handle);
        dev->i2c_dma_state = BME280_DMA_STATE_READING;
    }

  /* USER CODE END I2C1_EV_IRQn 0 */
}

/**
  * @brief This function handles I2C1 error interrupt.
  */
void I2C1_ER_IRQHandler(void)
{
  /* USER CODE BEGIN I2C1_ER_IRQn 0 */

    // AF (Acknowledge Failure) bayrağını temizle
    if (LL_I2C_IsActiveFlag_AF(I2C1))
    {
        LL_I2C_ClearFlag_AF(I2C1);
    }
    // BERR (Bus Error) bayrağını temizle
    if (LL_I2C_IsActiveFlag_BERR(I2C1))
    {
        LL_I2C_ClearFlag_BERR(I2C1);
    }
    // Diğer tüm olası hatalar...
    if (LL_I2C_IsActiveFlag_ARLO(I2C1)) { LL_I2C_ClearFlag_ARLO(I2C1); }
    if (LL_I2C_IsActiveFlag_OVR(I2C1)) { LL_I2C_ClearFlag_OVR(I2C1); }

    BME280_DMA_Cleanup(); // Tüm sistemi temizle ve sıfırla
    success = 0; // Başarısız oldu
  /* USER CODE END I2C1_ER_IRQn 0 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
