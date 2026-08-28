#include "ceva_spi_hal.h"

#include "sh2_err.h"

#include <string.h>

#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/spi.h>
#include <zephyr/kernel.h>

#define FSM_NODE DT_NODELABEL(fsm300)
#define FSM_TRANSFER_MAX SH2_HAL_MAX_TRANSFER_IN
#define FSM_RESET_DELAY_MS 10
#define FSM_START_DELAY_MS 200
#define FSM_WRITE_READY_TIMEOUT_MS 50

typedef struct {
    sh2_Hal_t api;
    struct gpio_callback interrupt_callback;
    uint8_t rx[FSM_TRANSFER_MAX];
    uint32_t interrupt_timestamp_us;
    volatile bool data_ready;
    bool opened;
} asr_fc_ceva_hal_t;

static asr_fc_ceva_hal_t hal;
static const struct spi_dt_spec spi = SPI_DT_SPEC_GET(
    FSM_NODE,
    SPI_WORD_SET(8) | SPI_TRANSFER_MSB | SPI_MODE_CPOL | SPI_MODE_CPHA |
        SPI_HOLD_ON_CS,
    0);
static const struct gpio_dt_spec reset_gpio =
    GPIO_DT_SPEC_GET(DT_ALIAS(fsm_reset), gpios);
static const struct gpio_dt_spec boot_gpio =
    GPIO_DT_SPEC_GET(DT_ALIAS(fsm_boot), gpios);
static const struct gpio_dt_spec wake_gpio =
    GPIO_DT_SPEC_GET(DT_ALIAS(fsm_wake), gpios);
static const struct gpio_dt_spec interrupt_gpio =
    GPIO_DT_SPEC_GET(DT_ALIAS(fsm_int), gpios);

static uint32_t now_us(void) {
    return k_cyc_to_us_floor32(k_cycle_get_32());
}

static void interrupt_handler(const struct device *device,
                              struct gpio_callback *callback,
                              uint32_t pins) {
    ARG_UNUSED(device);
    ARG_UNUSED(callback);
    ARG_UNUSED(pins);
    hal.interrupt_timestamp_us = now_us();
    hal.data_ready = true;
}

static int transceive(uint8_t *tx, uint8_t *rx, size_t length) {
    const struct spi_buf tx_buffer = {.buf = tx, .len = length};
    const struct spi_buf_set tx_set = {.buffers = &tx_buffer, .count = 1};
    struct spi_buf rx_buffer = {.buf = rx, .len = length};
    const struct spi_buf_set rx_set = {.buffers = &rx_buffer, .count = 1};
    return spi_transceive_dt(&spi, &tx_set, &rx_set);
}

static int read_transfer(uint8_t *buffer, unsigned capacity) {
    static uint8_t zeros[FSM_TRANSFER_MAX];
    uint8_t header[4] = {0};
    int result = transceive(zeros, header, sizeof(header));
    if (result != 0) {
        (void)spi_release(spi.bus, &spi.config);
        return SH2_ERR_IO;
    }

    const uint16_t length =
        (uint16_t)(((uint16_t)header[1] << 8u) | header[0]) & 0x7fffu;
    if (length < sizeof(header) || length > capacity ||
        length > FSM_TRANSFER_MAX) {
        (void)spi_release(spi.bus, &spi.config);
        return length == 0u ? 0 : SH2_ERR_BAD_PARAM;
    }
    memcpy(buffer, header, sizeof(header));
    if (length > sizeof(header)) {
        result = transceive(zeros, buffer + sizeof(header),
                            length - sizeof(header));
    }
    (void)spi_release(spi.bus, &spi.config);
    return result == 0 ? (int)length : SH2_ERR_IO;
}

static int ceva_open(sh2_Hal_t *self) {
    ARG_UNUSED(self);
    if (!spi_is_ready_dt(&spi) || !gpio_is_ready_dt(&reset_gpio) ||
        !gpio_is_ready_dt(&boot_gpio) || !gpio_is_ready_dt(&wake_gpio) ||
        !gpio_is_ready_dt(&interrupt_gpio)) {
        return SH2_ERR_IO;
    }
    if (gpio_pin_configure_dt(&reset_gpio, GPIO_OUTPUT_ACTIVE) != 0 ||
        gpio_pin_configure_dt(&boot_gpio, GPIO_OUTPUT_ACTIVE) != 0 ||
        gpio_pin_configure_dt(&wake_gpio, GPIO_OUTPUT_INACTIVE) != 0 ||
        gpio_pin_configure_dt(&interrupt_gpio, GPIO_INPUT) != 0) {
        return SH2_ERR_IO;
    }
    gpio_init_callback(&hal.interrupt_callback, interrupt_handler,
                       BIT(interrupt_gpio.pin));
    if (gpio_add_callback(interrupt_gpio.port, &hal.interrupt_callback) != 0 ||
        gpio_pin_interrupt_configure_dt(&interrupt_gpio,
                                        GPIO_INT_EDGE_TO_ACTIVE) != 0) {
        return SH2_ERR_IO;
    }

    hal.data_ready = false;
    k_sleep(K_MSEC(FSM_RESET_DELAY_MS));
    (void)gpio_pin_set_dt(&boot_gpio, 1);
    (void)gpio_pin_set_dt(&wake_gpio, 0);
    (void)gpio_pin_set_dt(&reset_gpio, 0);
    k_sleep(K_MSEC(FSM_START_DELAY_MS));
    hal.opened = true;
    return SH2_OK;
}

static void ceva_close(sh2_Hal_t *self) {
    ARG_UNUSED(self);
    (void)gpio_pin_interrupt_configure_dt(&interrupt_gpio, GPIO_INT_DISABLE);
    (void)gpio_pin_set_dt(&reset_gpio, 1);
    hal.opened = false;
    hal.data_ready = false;
}

static int ceva_read(sh2_Hal_t *self, uint8_t *buffer, unsigned length,
                     uint32_t *timestamp_us) {
    ARG_UNUSED(self);
    if (!hal.opened || buffer == NULL || timestamp_us == NULL) {
        return SH2_ERR_BAD_PARAM;
    }
    if (!hal.data_ready && gpio_pin_get_dt(&interrupt_gpio) == 0) {
        return 0;
    }
    hal.data_ready = false;
    *timestamp_us = hal.interrupt_timestamp_us;
    if (*timestamp_us == 0u) {
        *timestamp_us = now_us();
    }
    return read_transfer(buffer, length);
}

static int ceva_write(sh2_Hal_t *self, uint8_t *buffer, unsigned length) {
    ARG_UNUSED(self);
    if (!hal.opened || buffer == NULL || length == 0u ||
        length > FSM_TRANSFER_MAX) {
        return SH2_ERR_BAD_PARAM;
    }
    (void)gpio_pin_set_dt(&wake_gpio, 1);
    const int64_t deadline = k_uptime_get() + FSM_WRITE_READY_TIMEOUT_MS;
    while (gpio_pin_get_dt(&interrupt_gpio) == 0 && k_uptime_get() < deadline) {
        k_sleep(K_MSEC(1));
    }
    if (gpio_pin_get_dt(&interrupt_gpio) == 0) {
        (void)gpio_pin_set_dt(&wake_gpio, 0);
        return 0;
    }
    memset(hal.rx, 0, length);
    const int result = transceive(buffer, hal.rx, length);
    (void)spi_release(spi.bus, &spi.config);
    (void)gpio_pin_set_dt(&wake_gpio, 0);
    return result == 0 ? (int)length : SH2_ERR_IO;
}

static uint32_t ceva_time_us(sh2_Hal_t *self) {
    ARG_UNUSED(self);
    return now_us();
}

sh2_Hal_t *asr_fc_ceva_spi_hal(void) {
    hal.api.open = ceva_open;
    hal.api.close = ceva_close;
    hal.api.read = ceva_read;
    hal.api.write = ceva_write;
    hal.api.getTimeUs = ceva_time_us;
    return &hal.api;
}
