#include "asr_fc/protocol/motor_link.h"

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>

#define COMMAND_PERIOD_MS 20u
#define COMMAND_TIMEOUT_MS 60u
#define HEARTBEAT_TIMEOUT_MS 200u
#define HIL_SESSION_ID 0x41535231u

static const struct device *const command_uart =
    DEVICE_DT_GET(DT_NODELABEL(uart0));

static void write_frame(const uint8_t *frame, size_t frame_size) {
    for (size_t index = 0; index < frame_size; ++index) {
        uart_poll_out(command_uart, frame[index]);
    }
}

int main(void) {
    if (!device_is_ready(command_uart)) {
        return 1;
    }

    uint32_t sequence = 1u;
    int64_t next_release_ms = k_uptime_get();
    for (;;) {
        next_release_ms += COMMAND_PERIOD_MS;
        for (uint8_t motor = 0; motor < ASR_FC_MOTOR_COUNT; ++motor) {
            /* This image cannot arm a motor. Nonzero values exercise routing. */
            const asr_fc_motor_command_t command = {
                .motor_index = motor,
                .armed = false,
                .throttle_q15 = (uint16_t)(2000u * ((uint16_t)motor + 1u)),
                .session_id = HIL_SESSION_ID,
                .command_timeout_ms = COMMAND_TIMEOUT_MS,
                .heartbeat_timeout_ms = HEARTBEAT_TIMEOUT_MS,
            };
            uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
            size_t frame_size = 0u;
            if (asr_fc_motor_link_encode_command(
                    sequence++, &command, frame, sizeof(frame),
                    &frame_size) != ASR_FC_MOTOR_LINK_OK) {
                return 2;
            }
            write_frame(frame, frame_size);
        }
        const int64_t remaining_ms = next_release_ms - k_uptime_get();
        if (remaining_ms > 0) {
            k_sleep(K_MSEC(remaining_ms));
        } else {
            next_release_ms = k_uptime_get();
        }
    }
}
