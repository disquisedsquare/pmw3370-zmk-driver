// SPDX-License-Identifier: MIT
#include <zephyr/device.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>
#include <zephyr/logging/log.h>
#include <zmk/behavior.h>
#include <zmk/sensors.h>

LOG_MODULE_REGISTER(pmw3370, LOG_LEVEL_INF);

struct pmw3370_config {
    struct spi_dt_spec bus;
    struct gpio_dt_spec irq;
};

struct pmw3370_data {
    struct k_work work;
    const struct device *dev;
};

static void pmw3370_isr(const struct device *dev, struct gpio_callback *cb, uint32_t pins)
{
    struct pmw3370_data *data = CONTAINER_OF(cb, struct pmw3370_data, work);

    k_work_submit(&data->work);
}

static void pmw3370_work_handler(struct k_work *work)
{
    struct pmw3370_data *data = CONTAINER_OF(work, struct pmw3370_data, work);
    const struct pmw3370_config *cfg = data->dev->config;

    // Read motion data over SPI
    uint8_t tx_buf[3] = {0x02, 0, 0}; // example read command
    uint8_t rx_buf[3] = {0};

    struct spi_buf tx_spi_buf = {
        .buf = tx_buf,
        .len = sizeof(tx_buf)
    };
    struct spi_buf rx_spi_buf = {
        .buf = rx_buf,
        .len = sizeof(rx_buf)
    };
    struct spi_buf_set tx_set = { .buffers = &tx_spi_buf, .count = 1 };
    struct spi_buf_set rx_set = { .buffers = &rx_spi_buf, .count = 1 };

    int ret = spi_transceive_dt(&cfg->bus, &tx_set, &rx_set);
    if (ret) {
        LOG_ERR("SPI transfer failed: %d", ret);
        return;
    }

    int16_t dx = (int16_t)((rx_buf[0] << 8) | rx_buf[1]);
    int16_t dy = (int16_t)((rx_buf[2] << 8) | rx_buf[3]);

    LOG_INF("Motion dx=%d dy=%d", dx, dy);

    // TODO: push events to ZMK input transform system
}

static int pmw3370_init(const struct device *dev)
{
    struct pmw3370_data *data = dev->data;
    const struct pmw3370_config *cfg = dev->config;

    data->dev = dev;
    k_work_init(&data->work, pmw3370_work_handler);

    if (!device_is_ready(cfg->bus.bus)) {
        LOG_ERR("SPI device not ready");
        return -ENODEV;
    }

    if (!device_is_ready(cfg->irq.port)) {
        LOG_ERR("IRQ GPIO not ready");
        return -ENODEV;
    }

    int ret = gpio_pin_configure_dt(&cfg->irq, GPIO_INPUT);
    if (ret) {
        LOG_ERR("Failed to configure IRQ pin: %d", ret);
        return ret;
    }

    ret = gpio_pin_interrupt_configure_dt(&cfg->irq, GPIO_INT_EDGE_TO_ACTIVE);
    if (ret) {
        LOG_ERR("Failed to configure IRQ interrupt: %d", ret);
        return ret;
    }

    return 0;
}

#define PMW3370_DEFINE(inst)                                      \
    static struct pmw3370_data pmw3370_data_##inst;               \
    static const struct pmw3370_config pmw3370_config_##inst = {  \
        .bus = SPI_DT_SPEC_INST_GET(inst, SPI_WORD_SET(8) | SPI_TRANSFER_MSB, 0), \
        .irq = GPIO_DT_SPEC_INST_GET(inst, irq_gpios),            \
    };                                                            \
    DEVICE_DT_INST_DEFINE(inst, pmw3370_init, NULL,               \
        &pmw3370_data_##inst, &pmw3370_config_##inst,            \
        POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY, NULL);

DT_INST_FOREACH_STATUS_OKAY(PMW3370_DEFINE)

