/* pmw3370.c - PMW3370 Zephyr sensor driver (modern DT APIs)
 *
 * - Uses SPI controller at nodelabel "spi0" and the child node "pmw3370@0"
 * - Reads motion via IRQ (irq-gpios) and exposes dx/dy via sensor API
 * - SPI mode 3, up to spi-max-frequency on the child node (default 2 MHz)
 *
 * NOTE: The overlay MUST define a nodelabel "pmw3370" for the PMW3370 child:
 *   pmw3370: pmw3370@0 { compatible = "pixart,pmw3370"; reg = <0>; spi-max-frequency = <2000000>; irq-gpios = <&gpio0 6 GPIO_ACTIVE_LOW>; ... };
 */

#include <zephyr/kernel.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/device.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/logging/log.h>
#include "pmw3370.h"

LOG_MODULE_REGISTER(pmw3370, LOG_LEVEL_DBG);

struct pmw3370_data {
    const struct device *spi_dev;
    struct spi_config spi_cfg;
    struct gpio_callback irq_cb;
    struct gpio_dt_spec irq_spec;
    int16_t last_dx;
    int16_t last_dy;
    const struct device *dev;
    struct k_mutex lock;
};

/* Helper to write a register: write op = 0x80 | reg, then value */
static int pmw3370_write_reg(const struct device *dev, uint8_t reg, uint8_t val)
{
    struct pmw3370_data *d = dev->data;
    uint8_t tx[2] = { (uint8_t)(0x80U | reg), val };
    const struct spi_buf tx_buf = { .buf = tx, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };

    return spi_write(d->spi_dev, &d->spi_cfg, &tx_set);
}

/* Helper to read a register:
 * Some PixArt parts require sending address byte and then reading.
 * We send the address (reg & 0x7F) and read back 2 bytes; the data is in byte 1.
 */
static int pmw3370_read_reg(const struct device *dev, uint8_t reg, uint8_t *out)
{
    struct pmw3370_data *d = dev->data;
    uint8_t addr = reg & 0x7FU;
    uint8_t rx[2] = { 0, 0 };
    const struct spi_buf tx_buf = { .buf = &addr, .len = 1 };
    const struct spi_buf rx_buf = { .buf = rx, .len = 2 };
    const struct spi_buf_set tx_set = { .buffers = &tx_buf, .count = 1 };
    const struct spi_buf_set rx_set = { .buffers = &rx_buf, .count = 1 };

    int rc = spi_transceive(d->spi_dev, &d->spi_cfg, &tx_set, &rx_set);
    if (rc == 0) {
        *out = rx[1];
    }
    return rc;
}

/* Read Delta X/Y registers (0x03..0x06) as 16-bit signed (two's complement) */
static int pmw3370_read_motion(const struct device *dev, int16_t *dx, int16_t *dy)
{
    uint8_t xl, xh, yl, yh;
    int rc;

    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_X_L, &xl);
    if (rc) return rc;
    k_busy_wait(50);
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_X_H, &xh);
    if (rc) return rc;
    k_busy_wait(50);
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_Y_L, &yl);
    if (rc) return rc;
    k_busy_wait(50);
    rc = pmw3370_read_reg(dev, PMW_REG_DELTA_Y_H, &yh);
    if (rc) return rc;

    *dx = (int16_t)((xh << 8) | xl);
    *dy = (int16_t)((yh << 8) | yl);

    return 0;
}

/* Minimal power-up-init based on datasheet sequence. This implements the
 * concrete register writes provided earlier. If additional steps are required,
 * extend this function.
 */
static int pmw3370_powerup_init(const struct device *dev)
{
    /* A subset of the long initialization sequence; expand as needed. */
    pmw3370_write_reg(dev, 0x7F, 0x12);
    pmw3370_write_reg(dev, 0x47, 0x00);
    pmw3370_write_reg(dev, 0x7F, 0x00);
    pmw3370_write_reg(dev, 0x18, 0x00);
    pmw3370_write_reg(dev, 0x40, 0x80);
    /* ... other register writes from the sequence ... */
    pmw3370_write_reg(dev, 0x4D, 0x50);
    pmw3370_write_reg(dev, 0x4E, 0x3B);
    pmw3370_write_reg(dev, 0x4F, 0x46);
    pmw3370_write_reg(dev, 0x77, 0x24);
    pmw3370_write_reg(dev, 0x44, 0xA8);
    pmw3370_write_reg(dev, 0x46, 0x15);
    pmw3370_write_reg(dev, 0x4A, 0x14);
    pmw3370_write_reg(dev, 0x51, 0x10);
    pmw3370_write_reg(dev, 0x53, 0x0C);
    pmw3370_write_reg(dev, 0x55, 0xC9);
    pmw3370_write_reg(dev, 0x5B, 0xEA);
    pmw3370_write_reg(dev, 0x61, 0x13);
    pmw3370_write_reg(dev, 0x62, 0x0B);
    pmw3370_write_reg(dev, 0x64, 0x18);
    pmw3370_write_reg(dev, 0x6D, 0x86);
    pmw3370_write_reg(dev, 0x7D, 0x85);
    pmw3370_write_reg(dev, 0x7E, 0x03);
    pmw3370_write_reg(dev, 0x60, 0xB0);
    pmw3370_write_reg(dev, 0x6D, 0x29);
    pmw3370_write_reg(dev, 0x6E, 0x23);
    pmw3370_write_reg(dev, 0x42, 0x15);
    pmw3370_write_reg(dev, 0x40, 0x03);
    pmw3370_write_reg(dev, 0x4C, 0x28);
    pmw3370_write_reg(dev, 0x49, 0x00);
    pmw3370_write_reg(dev, 0x4F, 0x02);
    pmw3370_write_reg(dev, 0x53, 0x0C);
    pmw3370_write_reg(dev, 0x4A, 0x67);
    pmw3370_write_reg(dev, 0x6D, 0x20);
    pmw3370_write_reg(dev, 0x73, 0x83);
    pmw3370_write_reg(dev, 0x74, 0x00);
    pmw3370_write_reg(dev, 0x7A, 0x10);
    pmw3370_write_reg(dev, 0x63, 0x14);
    pmw3370_write_reg(dev, 0x4D, 0x4F);
    pmw3370_write_reg(dev, 0x4E, 0x1B);
    pmw3370_write_reg(dev, 0x54, 0x00);
    pmw3370_write_reg(dev, 0x55, 0x60);
    pmw3370_write_reg(dev, 0x56, 0x60);
    pmw3370_write_reg(dev, 0x58, 0x30);
    pmw3370_write_reg(dev, 0x59, 0x63);
    pmw3370_write_reg(dev, 0x4B, 0x23);
    pmw3370_write_reg(dev, 0x4C, 0x40);
    pmw3370_write_reg(dev, 0x4E, 0x6B);
    pmw3370_write_reg(dev, 0x5E, 0xC3);
    pmw3370_write_reg(dev, 0x4F, 0x02);
    pmw3370_write_reg(dev, 0x40, 0x90);
    pmw3370_write_reg(dev, 0x45, 0x1E);
    pmw3370_write_reg(dev, 0x46, 0xF0);
    pmw3370_write_reg(dev, 0x48, 0x0F);
    pmw3370_write_reg(dev, 0x49, 0x88);
    pmw3370_write_reg(dev, 0x4C, 0x15);
    pmw3370_write_reg(dev, 0x51, 0x6F);
    pmw3370_write_reg(dev, 0x52, 0x90);
    pmw3370_write_reg(dev, 0x54, 0x64);
    pmw3370_write_reg(dev, 0x55, 0xF0);
    pmw3370_write_reg(dev, 0x5C, 0x40);
    pmw3370_write_reg(dev, 0x61, 0xEE);
    pmw3370_write_reg(dev, 0x62, 0xE5);
    pmw3370_write_reg(dev, 0x53, 0x0C);
    pmw3370_write_reg(dev, 0x4A, 0x67);
    pmw3370_write_reg(dev, 0x6D, 0x20);
    pmw3370_write_reg(dev, 0x6E, 0x00);
    pmw3370_write_reg(dev, 0x73, 0x83);
    pmw3370_write_reg(dev, 0x74, 0x00);
    pmw3370_write_reg(dev, 0x7F, 0x00);
    pmw3370_write_reg(dev, 0x5B, 0x40);
    pmw3370_write_reg(dev, 0x61, 0xAD);
    pmw3370_write_reg(dev, 0x51, 0xEA);
    pmw3370_write_reg(dev, 0x19, 0x9F);

    for (int i = 0; i < 100; i++) {
        uint8_t v = 0;
        if (pmw3370_read_reg(dev, 0x20, &v) == 0 && v == 0x0F) {
            return 0;
        }
        k_msleep(1);
    }

    LOG_WRN("pmw3370 init: chip didn't report ready (0x20 != 0x0F)");
    return 0;
}

/* IRQ handler - the gpio callback passes us the gpio callback struct,
 * we get the driver data and read motion from the chip.
 */
static void pmw3370_irq_handler(const struct device *port, struct gpio_callback *cb, uint32_t pins)
{
    ARG_UNUSED(port);
    struct pmw3370_data *d = CONTAINER_OF(cb, struct pmw3370_data, irq_cb);
    int16_t dx = 0, dy = 0;

    if (pmw3370_read_motion(d->dev, &dx, &dy) == 0) {
        k_mutex_lock(&d->lock, K_NO_WAIT);
        d->last_dx = dx;
        d->last_dy = dy;
        k_mutex_unlock(&d->lock);
        LOG_DBG("pmw3370 irq dx=%d dy=%d", dx, dy);
    } else {
        LOG_DBG("pmw3370 irq read failed");
    }
}

/* sensor API: fetch called by ZMK to sample values */
static int pmw3370_sample_fetch(const struct device *dev, enum sensor_channel chan)
{
    struct pmw3370_data *d = dev->data;
    ARG_UNUSED(chan);

    int16_t dx = 0, dy = 0;
    int rc = pmw3370_read_motion(dev, &dx, &dy);
    if (rc == 0) {
        k_mutex_lock(&d->lock, K_FOREVER);
        d->last_dx = dx;
        d->last_dy = dy;
        k_mutex_unlock(&d->lock);
    }
    return rc;
}

static int pmw3370_channel_get(const struct device *dev, enum sensor_channel chan, struct sensor_value *val)
{
    struct pmw3370_data *d = dev->data;
    k_mutex_lock(&d->lock, K_FOREVER);
    if (chan == SENSOR_CHAN_ACCEL_X) {
        val->val1 = d->last_dx;
        val->val2 = 0;
    } else if (chan == SENSOR_CHAN_ACCEL_Y) {
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
    .channel_get = pmw3370_channel_get,
};

static int pmw3370_init(const struct device *dev)
{
    struct pmw3370_data *d = dev->data;

    d->dev = dev;

    /* Get SPI device by node label (spi0) */
    const struct device *spi_dev = DEVICE_DT_GET(DT_NODELABEL(spi0));
    if (!device_is_ready(spi_dev)) {
        LOG_ERR("pmw3370: spi0 device not ready");
        return -ENODEV;
    }
    d->spi_dev = spi_dev;

    /* Configure spi_config */
    d->spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_OP_MODE_MASTER | SPI_MODE_CPOL | SPI_MODE_CPHA;
    /* Pull slave index from child node 'reg' property (pmw3370@0 reg = <0>) */
    d->spi_cfg.slave = DT_REG_ADDR(DT_NODELABEL(pmw3370));
    /* Pull frequency from child node property spi-max-frequency (fallback 2000000) */
    d->spi_cfg.frequency = DT_PROP_OR(DT_NODELABEL(pmw3370), spi_max_frequency, 2000000);

    k_mutex_init(&d->lock);
    d->last_dx = 0;
    d->last_dy = 0;

    /* Setup IRQ gpio from the pmw3370 child node's irq-gpios */
    d->irq_spec = GPIO_DT_SPEC_GET(DT_NODELABEL(pmw3370), irq_gpios);
    if (device_is_ready(d->irq_spec.port)) {
        int rc = gpio_pin_configure_dt(&d->irq_spec, GPIO_INPUT | GPIO_PULL_UP);
        if (rc) {
            LOG_WRN("pmw3370: failed to configure irq pin (%d)", rc);
        } else {
            gpio_init_callback(&d->irq_cb, pmw3370_irq_handler, BIT(d->irq_spec.pin));
            gpio_add_callback(d->irq_spec.port, &d->irq_cb);
            gpio_pin_interrupt_configure_dt(&d->irq_spec, GPIO_INT_EDGE_TO_ACTIVE);
            LOG_INF("pmw3370: irq configured on %s pin %d", d->irq_spec.port->name, d->irq_spec.pin);
        }
    } else {
        LOG_WRN("pmw3370: irq gpio not ready - will poll instead");
    }

    /* Power-up reset & initialization */
    pmw3370_write_reg(dev, PMW_REG_POWER_UP_RESET, 0x00);
    k_msleep(5);
    pmw3370_powerup_init(dev);

    /* Probe product id */
    uint8_t pid = 0;
    if (pmw3370_read_reg(dev, PMW_REG_PRODUCT_ID, &pid) == 0) {
        LOG_INF("pmw3370 product id: 0x%02x", pid);
    } else {
        LOG_WRN("pmw3370: could not read product id");
    }

    return 0;
}

static struct pmw3370_data pmw3370_dev_data;

DEVICE_DEFINE(pmw3370, "PMW3370", pmw3370_init, NULL, &pmw3370_dev_data, NULL,
              APPLICATION, CONFIG_SENSOR_INIT_PRIORITY, &pmw3370_api);

