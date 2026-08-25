#include "asr_fc/protocol/motor_link.h"

#include "board.h"
#include "clock_config.h"
#include "fsl_uart.h"
#include "pin_mux.h"

#define SELECTED_MOTOR 0u
#define OUTPUT_INHIBITED_FAULT (1u << 31)

static void uart_write(const uint8_t *data, size_t size) {
    (void)UART_WriteBlocking(UART0, data, size);
}

int main(void) {
    BOARD_InitBootPins();
    BOARD_InitBootClocks();

    uart_config_t uart_config;
    UART_GetDefaultConfig(&uart_config);
    uart_config.baudRate_Bps = BOARD_DEBUG_UART_BAUDRATE;
    uart_config.enableTx = true;
    uart_config.enableRx = true;
    if (UART_Init(UART0, &uart_config, BOARD_DEBUG_UART_CLK_FREQ) !=
        kStatus_Success) {
        return 1;
    }

    asr_fc_motor_link_parser_t parser;
    asr_fc_motor_link_parser_init(&parser);
    uint32_t telemetry_sequence = 1u;
    for (;;) {
        if ((UART_GetStatusFlags(UART0) & kUART_RxDataRegFullFlag) == 0u) {
            continue;
        }
        const uint8_t byte = UART_ReadByte(UART0);
        uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
        size_t frame_size = 0u;
        bool frame_ready = false;
        if (asr_fc_motor_link_parser_push(
                &parser, byte, frame, sizeof(frame), &frame_size,
                &frame_ready) != ASR_FC_MOTOR_LINK_OK || !frame_ready) {
            continue;
        }

        uint32_t command_sequence = 0u;
        asr_fc_motor_command_t command;
        if (asr_fc_motor_link_decode_command(
                frame, frame_size, &command_sequence, &command) !=
                ASR_FC_MOTOR_LINK_OK ||
            command.motor_index != SELECTED_MOTOR) {
            continue;
        }

        /* PWM/FTM is intentionally absent from this image. */
        const asr_fc_motor_telemetry_t telemetry = {
            .motor_index = SELECTED_MOTOR,
            .motor_mode = 0u,
            .fault_flags = OUTPUT_INHIBITED_FAULT,
            .duty_q15 = 0u,
            .acknowledged_sequence = command_sequence,
        };
        size_t telemetry_size = 0u;
        if (asr_fc_motor_link_encode_telemetry(
                telemetry_sequence++, &telemetry, frame, sizeof(frame),
                &telemetry_size) == ASR_FC_MOTOR_LINK_OK) {
            uart_write(frame, telemetry_size);
        }
    }
}
