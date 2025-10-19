#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/input/input.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/util.h>

#include <zephyr/logging/log.h>
LOG_MODULE_REGISTER(pmw3370, CONFIG_SENSOR_LOG_LEVEL);

struct pmw3370_config {
    struct spi_dt_spec spi;
    struct gpio_dt_spec irq;
};

struct pmw3370_data {
    struct k_work work;
    int16_t dx, dy;
};

static uint8_t pmw3370_read(const struct device *dev, uint8_t reg) {
    const struct pmw3370_config *config = dev->config;
    uint8_t tx_buf[2] = {reg, 0};
    uint8_t rx_buf[2];
    const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};
    const struct spi_buf rx = {.buf = rx_buf, .len = sizeof(rx_buf)};
    const struct spi_buf_set rx_set = {.buffers = &rx, .count = 1};

    if (spi_transceive_dt(&config->spi, &tx_set, &rx_set)) {
        return 0;
    }
    return rx_buf[1];
}

static void pmw3370_write(const struct device *dev, uint8_t reg, uint8_t value) {
    const struct pmw3370_config *config = dev->config;
    uint8_t tx_buf[2] = {reg | 0x80, value};
    const struct spi_buf tx = {.buf = tx_buf, .len = sizeof(tx_buf)};
    const struct spi_buf_set tx_set = {.buffers = &tx, .count = 1};

    spi_write_dt(&config->spi, &tx_set);
}

static void pmw3370_burst_read(const struct device *dev, int16_t *dx, int16_t *dy) {
    uint8_t motion = pmw3370_read(dev, 0x02);
    if (motion & 0x80) {
        uint8_t xl = pmw3370_read(dev, 0x03);
        uint8_t xh = pmw3370_read(dev, 0x04);
        uint8_t yl = pmw3370_read(dev, 0x05);
        uint8_t yh = pmw3370_read(dev, 0x06);
        *dx = (int16_t)((xh << 8) | xl);
        *dy = (int16_t)((yh << 8) | yl);
        // Invert dy for right-hand trackball if needed (uncomment if motion is reversed)
        // *dy = -*dy;
    } else {
        *dx = 0;
        *dy = 0;
    }
}

static void pmw3370_work_handler(struct k_work *work) {
    struct pmw3370_data *data = CONTAINER_OF(work, struct pmw3370_data, work);
    const struct device *dev = CONTAINER_OF(data, struct device, data);
    int16_t dx, dy;

    pmw3370_burst_read(dev, &dx, &dy);

    if (dx != 0 || dy != 0) {
        input_report_rel(dev, INPUT_REL_X, dx);
        input_report_rel(dev, INPUT_REL_Y, dy);
        input_sync(dev);
    }
}

static void pmw3370_irq_callback(const struct device *port, struct gpio_callback *cb, uint32_t pins) {
    const struct device *dev = (const struct device *)cb->data;
    struct pmw3370_data *data = dev->data;
    k_work_submit(&data->work);
}

static int pmw3370_init(const struct device *dev) {
    const struct pmw3370_config *config = dev->config;
    struct pmw3370_data *data = dev->data;

    if (!spi_is_ready_dt(&config->spi)) {
        LOG_ERR("SPI not ready");
        return -ENODEV;
    }

    if (gpio_pin_configure_dt(&config->irq, GPIO_INPUT) < 0) {
        LOG_ERR("Failed to configure IRQ GPIO");
        return -EINVAL;
    }

    static struct gpio_callback cb_data;
    gpio_init_callback(&cb_data, pmw3370_irq_callback, BIT(config->irq.pin));
    cb_data.data = (void *)dev;
    if (gpio_add_callback(config->irq.port, &cb_data) < 0) {
        LOG_ERR("Failed to add GPIO callback");
        return -EINVAL;
    }

    if (gpio_pin_interrupt_configure_dt(&config->irq, GPIO_INT_EDGE_FALLING) < 0) {
        LOG_ERR("Failed to configure interrupt");
        return -EINVAL;
    }

    k_work_init(&data->work, pmw3370_work_handler);

    // Power-up sequence
    k_sleep(K_MSEC(50));

    // Reset SPI port: NCS high then low
    gpio_pin_set_dt(&config->spi.cs, 1);
    k_usleep(10);
    gpio_pin_set_dt(&config->spi.cs, 0);
    k_usleep(10);

    pmw3370_write(dev, 0x3A, 0x5A);
    k_sleep(K_MSEC(5));

    // Read registers once
    pmw3370_read(dev, 0x02);
    pmw3370_read(dev, 0x03);
    pmw3370_read(dev, 0x04);
    pmw3370_read(dev, 0x05);
    pmw3370_read(dev, 0x06);

    // Power-up initialization register settings
    pmw3370_write(dev, 0x7F, 0x12);
    pmw3370_write(dev, 0x47, 0x00);
    pmw3370_write(dev, 0x7F, 0x00);
    pmw3370_write(dev, 0x18, 0x00);
    pmw3370_write(dev, 0x40, 0x80);
    pmw3370_write(dev, 0x55, 0x01);
    k_sleep(K_MSEC(1));
    pmw3370_write(dev, 0x7F, 0x0E);
    pmw3370_write(dev, 0x43, 0x1D);
    uint8_t r1 = pmw3370_read(dev, 0x46);
    pmw3370_write(dev, 0x43, 0x1E);
    uint8_t r2 = pmw3370_read(dev, 0x46);
    pmw3370_write(dev, 0x7F, 0x14);
    pmw3370_write(dev, 0x6A, r1);
    pmw3370_write(dev, 0x6C, r2);
    pmw3370_write(dev, 0x7F, 0x00);
    pmw3370_write(dev, 0x55, 0x00);
    pmw3370_write(dev, 0x4D, 0x50);
    pmw3370_write(dev, 0x4E, 0x3B);
    pmw3370_write(dev, 0x4F, 0x46);
    pmw3370_write(dev, 0x54, 0x34);
    pmw3370_write(dev, 0x77, 0x24);
    pmw3370_write(dev, 0x7F, 0x05);
    pmw3370_write(dev, 0x44, 0xA8);
    pmw3370_write(dev, 0x46, 0x15);
    pmw3370_write(dev, 0x4A, 0x14);
    pmw3370_write(dev, 0x51, 0x10);
    pmw3370_write(dev, 0x53, 0x0C);
    pmw3370_write(dev, 0x55, 0xC9);
    pmw3370_write(dev, 0x5B, 0xEA);
    pmw3370_write(dev, 0x61, 0x13);
    pmw3370_write(dev, 0x62, 0x0B);
    pmw3370_write(dev, 0x64, 0x18);
    pmw3370_write(dev, 0x6D, 0x86);
    pmw3370_write(dev, 0x7D, 0x85);
    pmw3370_write(dev, 0x7E, 0x03);
    pmw3370_write(dev, 0x7F, 0x06);
    pmw3370_write(dev, 0x60, 0xB0);
    pmw3370_write(dev, 0x61, 0x00);
    pmw3370_write(dev, 0x6D, 0x29);
    pmw3370_write(dev, 0x6E, 0x23);
    pmw3370_write(dev, 0x7E, 0x40);
    pmw3370_write(dev, 0x7F, 0x07);
    pmw3370_write(dev, 0x42, 0x15);
    pmw3370_write(dev, 0x7F, 0x08);
    pmw3370_write(dev, 0x42, 0x28);
    pmw3370_write(dev, 0x43, 0x32);
    pmw3370_write(dev, 0x7F, 0x09);
    pmw3370_write(dev, 0x40, 0x03);
    pmw3370_write(dev, 0x7F, 0x0A);
    pmw3370_write(dev, 0x4A, 0x28);
    pmw3370_write(dev, 0x4C, 0x28);
    pmw3370_write(dev, 0x49, 0x00);
    pmw3370_write(dev, 0x4F, 0x02);
    pmw3370_write(dev, 0x7F, 0x0C);
    pmw3370_write(dev, 0x40, 0x90);
    pmw3370_write(dev, 0x41, 0x50);
    pmw3370_write(dev, 0x42, 0x0C);
    pmw3370_write(dev, 0x43, 0xA8);
    pmw3370_write(dev, 0x44, 0x47);
    pmw3370_write(dev, 0x45, 0x01);
    pmw3370_write(dev, 0x4D, 0x4F);
    pmw3370_write(dev, 0x4E, 0x1B);
    pmw3370_write(dev, 0x54, 0x00);
    pmw3370_write(dev, 0x55, 0x60);
    pmw3370_write(dev, 0x56, 0x60);
    pmw3370_write(dev, 0x58, 0x30);
    pmw3370_write(dev, 0x59, 0x63);
    pmw3370_write(dev, 0x7F, 0x0D);
    pmw3370_write(dev, 0x4B, 0x23);
    pmw3370_write(dev, 0x4C, 0x40);
    pmw3370_write(dev, 0x4E, 0x6B);
    pmw3370_write(dev, 0x5E, 0xC3);
    pmw3370_write(dev, 0x4F, 0x02);
    pmw3370_write(dev, 0x7F, 0x10);
    pmw3370_write(dev, 0x45, 0x1E);
    pmw3370_write(dev, 0x46, 0xF0);
    pmw3370_write(dev, 0x48, 0x0F);
    pmw3370_write(dev, 0x49, 0x88);
    pmw3370_write(dev, 0x4C, 0x15);
    pmw3370_write(dev, 0x4F, 0x00);
    pmw3370_write(dev, 0x51, 0x6F);
    pmw3370_write(dev, 0x52, 0x90);
    pmw3370_write(dev, 0x54, 0x64);
    pmw3370_write(dev, 0x55, 0xF0);
    pmw3370_write(dev, 0x5C, 0x40);
    pmw3370_write(dev, 0x61, 0xEE);
    pmw3370_write(dev, 0x62, 0xE5);
    pmw3370_write(dev, 0x7F, 0x14);
    pmw3370_write(dev, 0x53, 0x0C);
    pmw3370_write(dev, 0x4A, 0x67);
    pmw3370_write(dev, 0x6D, 0x20);
    pmw3370_write(dev, 0x6E, 0x00);
    pmw3370_write(dev, 0x73, 0x83);
    pmw3370_write(dev, 0x74, 0x00);
    pmw3370_write(dev, 0x7A, 0x16);
    pmw3370_write(dev, 0x63, 0x14);
    pmw3370_write(dev, 0x62, 0x14);
    pmw3370_write(dev, 0x7F, 0x00);
    pmw3370_write(dev, 0x5B, 0x40);
    pmw3370_write(dev, 0x61, 0xAD);
    pmw3370_write(dev, 0x51, 0xEA);
    pmw3370_write(dev, 0x19, 0x9F);

    // Read 0x20 until 0x0F, up to 100ms at 1ms intervals
    for (int i = 0; i < 100; i++) {
        k_sleep(K_MSEC(1));
        if (pmw3370_read(dev, 0x20) == 0x0F) {
            break;
        }
    }

    pmw3370_write(dev, 0x19, 0x10);
    pmw3370_write(dev, 0x61, 0xD5);
    pmw3370_write(dev, 0x40, 0x00);

    // Set CPI (resolution register 0x4E)
    uint8_t cpi_val = (CONFIG_PMW3370_CPI / 100) - 1;  // Assuming step of 100
    pmw3370_write(dev, 0x4E, cpi_val);

    LOG_INF("PMW3370 initialized");

    return 0;
}

INPUT_DT_DEFINE(pmw3370, pmw3370_init, NULL, &pmw3370_data, &pmw3370_config, POST_KERNEL, CONFIG_SENSOR_INIT_PRIORITY);
