#ifndef ASR_FC_PROTOCOL_MOTOR_LINK_H
#define ASR_FC_PROTOCOL_MOTOR_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ASR_FC_MOTOR_LINK_VERSION 1u
#define ASR_FC_MOTOR_LINK_MAX_PAYLOAD 32u
#define ASR_FC_MOTOR_LINK_HEADER_SIZE 10u
#define ASR_FC_MOTOR_LINK_CRC_SIZE 2u
#define ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE \
    (ASR_FC_MOTOR_LINK_HEADER_SIZE + ASR_FC_MOTOR_LINK_MAX_PAYLOAD + \
     ASR_FC_MOTOR_LINK_CRC_SIZE)

typedef enum {
    ASR_FC_MOTOR_LINK_COMMAND = 1,
    ASR_FC_MOTOR_LINK_TELEMETRY = 2,
    ASR_FC_MOTOR_LINK_HEARTBEAT = 3
} asr_fc_motor_link_type_t;

typedef enum {
    ASR_FC_MOTOR_LINK_OK = 0,
    ASR_FC_MOTOR_LINK_NULL_POINTER,
    ASR_FC_MOTOR_LINK_BUFFER_TOO_SMALL,
    ASR_FC_MOTOR_LINK_INVALID_FRAME,
    ASR_FC_MOTOR_LINK_CRC_FAILURE,
    ASR_FC_MOTOR_LINK_VERSION_MISMATCH
} asr_fc_motor_link_status_t;

typedef struct {
    uint8_t motor_index;
    bool armed;
    uint16_t flags;
    uint16_t throttle_q15;
    uint32_t session_id;
    uint16_t command_timeout_ms;
    uint16_t heartbeat_timeout_ms;
} asr_fc_motor_command_t;

typedef struct {
    uint8_t motor_index;
    uint8_t motor_mode;
    uint32_t fault_flags;
    uint32_t bus_voltage_mv;
    int32_t dc_current_ma;
    uint32_t electrical_period_us;
    uint16_t duty_q15;
    uint32_t acknowledged_sequence;
} asr_fc_motor_telemetry_t;

typedef struct {
    uint8_t buffer[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t size;
    size_t expected_size;
} asr_fc_motor_link_parser_t;

void asr_fc_motor_link_parser_init(asr_fc_motor_link_parser_t *parser);

asr_fc_motor_link_status_t asr_fc_motor_link_parser_push(
    asr_fc_motor_link_parser_t *parser,
    uint8_t byte,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size,
    bool *frame_ready);

uint16_t asr_fc_motor_link_crc16(const uint8_t *data, size_t size);

asr_fc_motor_link_status_t asr_fc_motor_link_encode_command(
    uint32_t sequence,
    const asr_fc_motor_command_t *command,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size);

asr_fc_motor_link_status_t asr_fc_motor_link_decode_command(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_motor_command_t *command);

asr_fc_motor_link_status_t asr_fc_motor_link_encode_telemetry(
    uint32_t sequence,
    const asr_fc_motor_telemetry_t *telemetry,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size);

asr_fc_motor_link_status_t asr_fc_motor_link_decode_telemetry(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_motor_telemetry_t *telemetry);

#endif
