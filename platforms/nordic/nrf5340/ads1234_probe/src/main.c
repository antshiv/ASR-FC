#include <stdint.h>

#include <zephyr/device.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define ADS1234_READY_TIMEOUT_MS 1500
#define ADS1234_BITS 24u

#define USER_NODE DT_PATH(zephyr_user)

static const struct gpio_dt_spec ads_sclk =
    GPIO_DT_SPEC_GET(USER_NODE, ads1234_sclk_gpios);
static const struct gpio_dt_spec ads_dout =
    GPIO_DT_SPEC_GET(USER_NODE, ads1234_dout_gpios);
static const struct gpio_dt_spec ads_pdwn =
    GPIO_DT_SPEC_GET(USER_NODE, ads1234_pdwn_gpios);

static void reset_adc(void) {
    /* TI requires high-low-high after both supply rails have stabilized. */
    gpio_pin_set_dt(&ads_pdwn, 1);
    k_busy_wait(50);
    gpio_pin_set_dt(&ads_pdwn, 0);
    k_busy_wait(50);
    gpio_pin_set_dt(&ads_pdwn, 1);
}

static int wait_for_ready(void) {
    const int64_t deadline = k_uptime_get() + ADS1234_READY_TIMEOUT_MS;

    while (k_uptime_get() < deadline) {
        const int level = gpio_pin_get_dt(&ads_dout);
        if (level < 0) {
            return level;
        }
        if (level == 0) {
            return 0;
        }
        k_sleep(K_MSEC(1));
    }

    return -ETIMEDOUT;
}

static int read_sample(int32_t *sample, uint32_t *raw_sample) {
    int status = wait_for_ready();
    if (status != 0) {
        return status;
    }

    uint32_t raw = 0u;
    for (uint32_t bit = 0u; bit < ADS1234_BITS; ++bit) {
        gpio_pin_set_dt(&ads_sclk, 1);
        k_busy_wait(1);

        const int level = gpio_pin_get_dt(&ads_dout);
        if (level < 0) {
            gpio_pin_set_dt(&ads_sclk, 0);
            return level;
        }
        raw = (raw << 1u) | (uint32_t)level;

        gpio_pin_set_dt(&ads_sclk, 0);
        k_busy_wait(1);
    }

    /* The optional 25th clock forces DRDY/DOUT high while conversion resumes. */
    gpio_pin_set_dt(&ads_sclk, 1);
    k_busy_wait(1);
    gpio_pin_set_dt(&ads_sclk, 0);

    *raw_sample = raw;
    *sample = (raw & 0x00800000u) != 0u
                  ? (int32_t)(raw | 0xff000000u)
                  : (int32_t)raw;
    return 0;
}

int main(void) {
    if (!gpio_is_ready_dt(&ads_sclk) || !gpio_is_ready_dt(&ads_dout) ||
        !gpio_is_ready_dt(&ads_pdwn)) {
        printk("ADS1234 ERROR gpio-not-ready\n");
        return 1;
    }

    int status = gpio_pin_configure_dt(&ads_sclk, GPIO_OUTPUT_INACTIVE);
    if (status != 0) {
        printk("ADS1234 ERROR sclk-config status=%d\n", status);
        return 2;
    }
    status = gpio_pin_configure_dt(&ads_dout, GPIO_INPUT);
    if (status != 0) {
        printk("ADS1234 ERROR dout-config status=%d\n", status);
        return 3;
    }
    status = gpio_pin_configure_dt(&ads_pdwn, GPIO_OUTPUT_INACTIVE);
    if (status != 0) {
        printk("ADS1234 ERROR pdwn-config status=%d\n", status);
        return 4;
    }

    k_sleep(K_MSEC(2));
    reset_adc();

    printk("ADS1234 PROBE pins=sclk:P1.03,dout:P1.02,pdwn:P1.04 "
           "mode=ch1,gain128,10sps\n");

    uint32_t sequence = 0u;
    for (;;) {
        int32_t sample = 0;
        uint32_t raw_sample = 0u;
        status = read_sample(&sample, &raw_sample);
        if (status == -ETIMEDOUT) {
            printk("ADS1234 TIMEOUT seq=%u dout=%d\n", sequence,
                   gpio_pin_get_dt(&ads_dout));
        } else if (status != 0) {
            printk("ADS1234 ERROR seq=%u status=%d\n", sequence, status);
        } else {
            printk("ADS1234 SAMPLE seq=%u ms=%lld raw=0x%06x counts=%d\n",
                   sequence, k_uptime_get(), raw_sample, sample);
        }
        ++sequence;
    }
}
