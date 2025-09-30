/* pmw3370.c - PMW3370 Zephyr sensor driver
 *
 * - Modern DT_INST_ APIs
 * - Uses spi_dt_spec for bus + chip select
 * - Reads motion via IRQ (irq-gpios) if provided
 * - Exposes dx/dy via sensor API
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "../../../include/pmw3370.h"

LOG_MODULE_REGISTER(pmw3370, CONFIG_SENSOR_LOG_LEVEL);

#define DT_DRV_COMPAT pixart_pmw3370

struct pmw3370_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq;
};

struct pmw3370_data {
    int16_t last_dx;
    int16_t last_dy;
    struct gpio_callback irq_cb;
    struct k_mutex lock;
};

/* --- low-level helpers --- */

static int pmw3370_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    const struct pmw3370_config *cfg = dev->config;
    uint8_t tx[2] = { (uint8_t)(0x80U | reg), val };
    struct spi_buf tx_buf = { .buf = tx, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

    return spi_write_dt(&cfg->bus, &tx_set);
}

static int pmw3370_read_reg(const struct device *dev, uint8_t reg, uint8_t *out)
{
    const struct pmw3370_config *cfg = dev->config;
    uint8_t addr = reg & 0x7F;
    uint8_t rx[2] = { 0 };

    struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    struct spi_buf rx_buf = { .buf = rx, .len = 2 };
    struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    int rc = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
    if (rc == 0) {
        *out = rx[1];
    }
    return rc;
}

static int pmw3370_read_motion(const struct device *dev, int16_t *dx, int16_t *dy)
{
    uint8_t xl, xh, yl, yh;
    int rc;

    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_X_L, &xl);
    if (rc) return rc;
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_X_H, &xh);
    if (rc) return rc;
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_Y_L, &yl);
    if (rc) return rc;
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_Y_H, &yh);
    if (rc) return rc;

    *dx = (int16_t)((xh << 8) | xl);
    *dy = (int16_t)((yh << 8) | yl);

    return 0;
}

/* --- sensor API --- */

static int pmw3370_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct pmw3370_data *d = dev->data;
    int16_t dx = 0, dy = 0;

    ARG_UNUSED(chan);

    int rc = pmw3370_read_motion(dev, &dx, &dy);
    if (rc == 0) {
        k_mutex_lock(&d->lock, K_FOREVER);
        d->last_dx = dx;
        d->last_dy = dy;
        k_mutex_unlock(&d->lock);
    }
    return rc;
}

static int pmw3370_channel_get(const struct device *dev,
                               enum sensor_channel chan,
                               struct sensor_value *val)
{
    struct pmw3370_data *d = dev->data;

    k_mutex_lock(&d->lock, K_FOREVER);
    if (chan == SENSOR_CHAN_POS_DX) {
        val->val1 = d->last_dx;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_POS_DY) {
        val->val1 = d->last_dy;
        val->val2 = 0;
    } else {
        k_mutex_unlock(&d->lock);
        return -ENOTSUP;
    }
    k_mutex_unlock(&d->lock);
    return 0;
}

static const struct sensor_driver_api pmw3370_api = {
    .sample_fetch = pmw3370_sample_fetch,
    .channel_get  = pmw3370_channel_get,
};

/* --- IRQ handling --- */

static void pmw3370_irq_handler(const struct device *port,
                                struct gpio_callback *cb,
                                uint32_t pins)
{
    ARG_UNUSED(port);
    ARG_UNUSED(pins);

    struct pmw3370_data *d = CONTAINER_OF(cb, struct pmw3370_data, irq_cb);
    const struct device *dev = DEVICE_DT_INST_GET(0); /* safe because 1 inst */
    int16_t dx, dy;

    if (pmw3370_read_motion(dev, &dx, &dy) == 0) {
        k_mutex_lock(&d->lock, K_NO_WAIT);
        d->last_dx = dx;
        d->last_dy = dy;
        k_mutex_unlock(&d->lock);
        LOG_DBG("irq dx=%d dy=%d", dx, dy);
    }
}

/* --- init --- */

static int pmw3370_init(const struct device *dev)
{
    const struct pmw3370_config *cfg = dev->config;
    struct pmw3370_data *d = dev->data;

    if (!spi_is_ready_dt(&cfg->bus)) {
        LOG_ERR("SPI bus not ready");
        return -ENODEV;
    }

    k_mutex_init(&d->lock);
    d->last_dx = d->last_dy = 0;

    if (cfg->irq.port && device_is_ready(cfg->irq.port)) {
        int rc = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
        if (rc == 0) {
            gpio_init_callback(&d->irq_cb, pmw3370_irq_handler, BIT(cfg->irq.pin));
            gpio_add_callback(cfg->irq.port, &d->irq_cb);
            gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
            LOG_INF("IRQ configured on %s pin %d", cfg->irq.port->name, cfg->irq.pin);
        }
    }

    /* reset + init sequence */
    pmw3370_write_reg(dev, PMW_REG_POWER_UP_RESET, 0x00);
    k_msleep(5);

    uint8_t pid = 0;
    if (pmw3370_read_reg(dev, PMW_REG_PRODUCT_ID, &pid) == 0) {
        LOG_INF("Product ID: 0x%02x", pid);
    }

    return 0;
}

/* --- device instantiation --- */

#define PMW3370_DEFINE(inst)                                              \
    static struct pmw3370_data pmw3370_data_##inst;                       \
    static const struct pmw3370_config pmw3370_config_##inst = {          \
        .bus = SPI_DT_SPEC_INST_GET(inst,                                 \
            SPI_WORD_SET(8) | SPI_TRANSFER_MSB |                          \
            SPI_MODE_CPOL | SPI_MODE_CPHA, 0),                            \
        .irq = GPIO_DT_SPEC_INST_GET_OR(inst, irq_gpios, {0}),            \
    };                                                                    \
    DEVICE_DT_INST_DEFINE(inst,                                           \
        pmw3370_init, NULL,                                               \
        &pmw3370_data_##inst, &pmw3370_config_##inst,                     \
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY,                         \
        &pmw3370_api);

DT_INST_FOREACH_STATUS_OKAY(PMW3370_DEFINE)

