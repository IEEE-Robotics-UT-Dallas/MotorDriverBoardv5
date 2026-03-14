/*
 * BNO085.c
 *
 * Driver for the Hillcrest/CEVA BNO085 IMU over SPI on STM32H5
 * using the CEVA sh2 sensor hub driver and FreeRTOS.
 *
 * Currently, DMA is not implemented interrupt is polled every 1ms
 *
 * Library written by Claude Sonnet 4.6
 */


#include "bno085.h"
#include "core_cm33.h"
#include <string.h>

static BNO085_t *s_dev = NULL;

// ── Logging Macro ─────────────────────────────────────────────────────────────
#define BNO_LOG(fmt, ...) ((void)0)

// ── DWT microsecond timer ─────────────────────────────────────────────────────
static void dwt_init(void) {
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0;
    DWT->CTRL  |= DWT_CTRL_CYCCNTENA_Msk;
}

static uint32_t dwt_get_us(void) {
    return DWT->CYCCNT / 250; // 250 MHz
}

// ── CS / RST ──────────────────────────────────────────────────────────────────
static inline void cs_low(void)   { HAL_GPIO_WritePin(BNO085_CS_PORT,  BNO085_CS_PIN,  GPIO_PIN_RESET); }
static inline void cs_high(void)  { HAL_GPIO_WritePin(BNO085_CS_PORT,  BNO085_CS_PIN,  GPIO_PIN_SET);   }
static inline void rst_low(void)  { HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_RESET); }
static inline void rst_high(void) { HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);   }

// ── INT ───────────────────────────────────────────────────────────────────────
void BNO085_INT_Callback(BNO085_t *dev) {
    dev->intFired = true;
}

static bool wait_for_int(void) {
    uint32_t start = HAL_GetTick();
    while (!s_dev->intFired) {
        // Also check the pin directly in case we missed the edge
        if (HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN) == GPIO_PIN_RESET) {
            s_dev->intFired = false;
            return true;
        }
        if ((HAL_GetTick() - start) > 500) {
            BNO_LOG("wait_for_int TIMEOUT (pin=%d)",
                    HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN));
            rst_low();
            osDelay(10);
            rst_high();
            osDelay(50);
            return false;
        }
        osDelay(1);
    }
    s_dev->intFired = false;
    return true;
}

// ── SPI ───────────────────────────────────────────────────────────────────────
static bool spi_transfer(uint8_t *buf, size_t len) {
    uint32_t start = HAL_GetTick();
    while (s_dev->hspi->State != HAL_SPI_STATE_READY) {
        if ((HAL_GetTick() - start) > 100) {
            BNO_LOG("spi_transfer: SPI stuck, aborting");
            HAL_SPI_Abort(s_dev->hspi);
            return false;
        }
        osDelay(1);
    }

    cs_low();
    HAL_StatusTypeDef r = HAL_SPI_TransmitReceive(s_dev->hspi, buf, buf, len, 100);
    cs_high();

    if (r != HAL_OK) {
        BNO_LOG("SPI error: %d (state=%d)", r, s_dev->hspi->State);
        HAL_SPI_Abort(s_dev->hspi);
        return false;
    }
    BNO_LOG("SPI rx[0..%d]: %02X %02X %02X %02X %02X %02X %02X %02X",
            (int)len - 1,
            buf[0], buf[1], buf[2], buf[3],
            len > 4 ? buf[4] : 0,
            len > 5 ? buf[5] : 0,
            len > 6 ? buf[6] : 0,
            len > 7 ? buf[7] : 0);
    return true;
}

// ── sh2 HAL ───────────────────────────────────────────────────────────────────
static int spihal_open(sh2_Hal_t *self) {
    BNO_LOG("spihal_open");
    // Do NOT clear intFired here — the INT may have already fired
    // during the reset delay and we don't want to miss it
    if (!wait_for_int()) {
        BNO_LOG("spihal_open: INT never fired");
        return -1;
    }
    BNO_LOG("spihal_open OK");
    return 0;
}

static void spihal_close(sh2_Hal_t *self) { }

static int spihal_read(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len,
                       uint32_t *t_us) {
    BNO_LOG("spihal_read: waiting for INT (header)");
    if (!wait_for_int()) return 0;

    memset(pBuffer, 0, 4);
    if (!spi_transfer(pBuffer, 4)) return 0;

    uint16_t packet_size = ((uint16_t)pBuffer[0] | ((uint16_t)pBuffer[1] << 8))
                           & ~0x8000;
    BNO_LOG("spihal_read: packet_size=%u", packet_size);

    if (packet_size == 0 || packet_size > len) {
        BNO_LOG("spihal_read: bad packet_size (buf len=%u)", len);
        return 0;
    }

    BNO_LOG("spihal_read: waiting for INT (payload)");
    if (!wait_for_int()) return 0;

    memset(pBuffer, 0, packet_size);
    if (!spi_transfer(pBuffer, packet_size)) return 0;

    BNO_LOG("spihal_read: done, channel=%u seq=%u", pBuffer[2], pBuffer[3]);

    if (t_us) *t_us = dwt_get_us();
    return (int)packet_size;
}

static int spihal_write(sh2_Hal_t *self, uint8_t *pBuffer, unsigned len) {
    BNO_LOG("spihal_write: len=%u", len);
    if (!wait_for_int()) return 0;
    if (!spi_transfer(pBuffer, len)) return 0;
    BNO_LOG("spihal_write: done");
    return (int)len;
}

static uint32_t hal_get_time_us(sh2_Hal_t *self) {
    return dwt_get_us();
}

// ── sh2 callbacks ─────────────────────────────────────────────────────────────
static void hal_callback(void *cookie, sh2_AsyncEvent_t *pEvent) {
    BNO_LOG("hal_callback: eventId=%u", pEvent->eventId);
    if (pEvent->eventId == SH2_RESET && s_dev)
        s_dev->resetOccurred = true;
}

static void sensor_handler(void *cookie, sh2_SensorEvent_t *event) {
    if (s_dev)
        sh2_decodeSensorEvent(&s_dev->sensorValue, event);
}

// ── Public API ────────────────────────────────────────────────────────────────
bool BNO085_Init(BNO085_t *dev, SPI_HandleTypeDef *hspi) {
    memset(dev, 0, sizeof(*dev));
    dev->hspi = hspi;
    s_dev = dev;

    dwt_init();

    BNO_LOG("Init: resetting hardware");
    cs_high();
    rst_low();
    osDelay(10);
    rst_high();
    osDelay(300);

    dev->hal.open      = spihal_open;
    dev->hal.close     = spihal_close;
    dev->hal.read      = spihal_read;
    dev->hal.write     = spihal_write;
    dev->hal.getTimeUs = hal_get_time_us;

    BNO_LOG("Init: calling sh2_open");
    if (sh2_open(&dev->hal, hal_callback, NULL) != SH2_OK) {
        BNO_LOG("Init: sh2_open FAILED");
        return false;
    }
    BNO_LOG("Init: sh2_open OK");

    sh2_ProductIds_t prodIds;
    memset(&prodIds, 0, sizeof(prodIds));
    BNO_LOG("Init: calling sh2_getProdIds");
    int status = SH2_ERR;
    for (int attempt = 0; attempt < 5; attempt++) {
        status = sh2_getProdIds(&prodIds);
        BNO_LOG("Init: getProdIds attempt %d status=%d", attempt, status);
        if (status == SH2_OK) break;
        osDelay(100);
    }
    if (status != SH2_OK) {
        BNO_LOG("Init: sh2_getProdIds FAILED after retries");
        return false;
    }

    BNO_LOG("Init: part=%d ver=%d.%d.%d",
            prodIds.entry[0].swPartNumber,
            prodIds.entry[0].swVersionMajor,
            prodIds.entry[0].swVersionMinor,
            prodIds.entry[0].swVersionPatch);

    sh2_setSensorCallback(sensor_handler, NULL);
    BNO_LOG("Init: complete");
    return true;
}

static bool enable_report(sh2_SensorId_t id, uint32_t interval_us) {
    sh2_SensorConfig_t cfg = {0};
    cfg.reportInterval_us = interval_us;
    return sh2_setSensorConfig(id, &cfg) == SH2_OK;
}

bool BNO085_EnableRotationVector(BNO085_t *dev, uint32_t interval_ms) {
    return enable_report(SH2_ROTATION_VECTOR, interval_ms * 1000);
}

bool BNO085_EnableAccelerometer(BNO085_t *dev, uint32_t interval_ms) {
    return enable_report(SH2_ACCELEROMETER, interval_ms * 1000);
}

bool BNO085_EnableGyro(BNO085_t *dev, uint32_t interval_ms) {
    return enable_report(SH2_GYROSCOPE_CALIBRATED, interval_ms * 1000);
}

bool BNO085_EnableGameRotationVector(BNO085_t *dev, uint32_t interval_ms) {
    return enable_report(SH2_GAME_ROTATION_VECTOR, interval_ms * 1000);
}

bool BNO085_GetSensorEvent(BNO085_t *dev) {
    dev->sensorValue.timestamp = 0;
    sh2_service();
    return dev->sensorValue.timestamp != 0 ||
           dev->sensorValue.sensorId == SH2_GYRO_INTEGRATED_RV;
}

bool BNO085_WasReset(BNO085_t *dev) {
    bool r = dev->resetOccurred;
    dev->resetOccurred = false;
    return r;
}
