#define LOG_LEVEL CONFIG_PMW3370_LOG_LEVEL
#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/logging/log.h>
#include "pmw3370.h"

LOG_MODULE_REGISTER(pmw3370, LOG_LEVEL);

struct pmw3370_data {
    const struct device *spi;
    struct spi_config spi_cfg;
    const struct device *irq_dev;
    int16_t last_dx;
    int16_t last_dy;
    struct k_mutex lock;
};

static int pmw3370_spi_read_reg(const struct device *dev, uint8_t reg, uint8_t *val)
{
    struct pmw3370_data *drv = dev->data;
    uint8_t tx[2] = { (uint8_t)(0x80 | reg), 0x00 };
    uint8_t rx[2] = {0};
    const struct spi_buf tx_buf = { .buf = tx, .len = 2 };
    const struct spi_buf rx_buf = { .buf = rx, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    int rc = spi_transceive(drv->spi, &drv->spi_cfg, &tx_set, &rx_set);
    if (rc) {
        LOG_ERR("spi_transceive reg 0x%02x failed: %d", reg, rc);
        return rc;
    }
    /* PMW33xx returns second byte as reg data */
    *val = rx[1];
    return 0;
}

static int pmw3370_spi_read_motion(const struct device *dev, int16_t *dx, int16_t *dy)
{
    struct pmw3370_data *drv = dev->data;
    uint8_t tx[5] = { (uint8_t)(0x80 | PMW_REG_MOTION), 0, 0, 0, 0 };
    uint8_t rx[5] = {0};
    const struct spi_buf tx_buf = { .buf = tx, .len = 5 };
    const struct spi_buf rx_buf = { .buf = rx, .len = 5 };
    const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    int rc = spi_transceive(drv->spi, &drv->spi_cfg, &tx_set, &rx_set);
    if (rc) {
        LOG_ERR("spi_transceive motion failed: %d", rc);
        return rc;
    }

    /* Note: PMW33xx motion packet parsing is sensor-specific. This is
     * simplified: combine low and high bytes for dx/dy if device uses that.
     * Many PMW chips: DELTA_X_L/DELTA_X_H are contiguous after MOTION.
     */
    uint8_t dx_l = rx[2];
    uint8_t dx_h = rx[3];
    uint8_t dy_l = rx[4];
    /* If you need more bytes, adjust length above and parsing accordingly. */

    /* Build signed 16-bit */
    int16_t raw_dx = (int16_t)((dx_h << 8) | dx_l);
    int16_t raw_dy = (int16_t)(dy_l); /* placeholder; extend as needed */

    *dx = raw_dx;
    *dy = raw_dy;

    return 0;
}

/* sensor API callbacks */
static int pmw3370_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct pmw3370_data *drv = dev->data;

    ARG_UNUSED(chan);

    k_mutex_lock(&drv->lock, K_FOREVER);
    int r = pmw3370_spi_read_motion(dev, &drv->last_dx, &drv->last_dy);
    k_mutex_unlock(&drv->lock);

    return r;
}

static int pmw3370_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct pmw3370_data *drv = dev->data;

    if (chan != SENSOR_CHAN_ACCEL_X && chan != SENSOR_CHAN_ACCEL_Y) {
        return -ENOTSUP;
    }

    /* Expose dx/dy as integer sensor values in milli-units (simple mapping) */
    k_mutex_lock(&drv->lock, K_FOREVER);
    if (chan == SENSOR_CHAN_ACCEL_X) {
        val->val1 = drv->last_dx;
        val->val2 = 0;
    } else {
        val->val1 = drv->last_dy;
        val->val2 = 0;
    }
    k_mutex_unlock(&drv->lock);

    return 0;
}

static const struct sensor_driver_api pmw3370_api = {
    .sample_fetch = pmw3370_sample_fetch,
    .channel_get = pmw3370_channel_get,
};

static int pmw3370_init(const struct device *dev)
{
    struct pmw3370_data *drv = dev->data;

    drv->spi = device_get_binding(DT_LABEL(DT_BUS(DT_NODELABEL(spi0))));
    if (!drv->spi) {
        LOG_ERR("Failed to get SPI device");
        return -ENODEV;
    }

    /* Example SPI config. The board overlay should provide proper
     * cs / spi controller configuration. We set mode and frequency here.
     */
    drv->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER;
    drv->spi_cfg.frequency = 2000000u;
    drv->spi_cfg.slave = 0; /* actual 'reg' value should match overlay */

    k_mutex_init(&drv->lock);
    drv->last_dx = 0;
    drv->last_dy = 0;

    /* Probe: read product id */
    uint8_t id = 0;
    if (pmw3370_spi_read_reg(dev, PMW_REG_PRODUCT_ID, &id) == 0) {
        LOG_INF("PMW product id: 0x%02x", id);
    } else {
        LOG_WRN("Could not read product id - check wiring and CS/IRQ settings");
    }

    /* TODO: upload SROM firmware here if needed, and any register initialization */

    return 0;
}

/* device instance data and declaration */
static struct pmw3370_data pmw3370_data = {
    .spi = NULL,
};

DEVICE_DEFINE(pmw3370, "PMW3370", pmw3370_init, NULL,
              &pmw3370_data, NULL, APPLICATION,
              CONFIG_SENSOR_INIT_PRIORITY, &pmw3370_api);

