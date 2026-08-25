#include "asr_fc/protocol/motor_link.h"

#include <string.h>

#define ASR_FC_MAGIC_0 ((uint8_t)'A')
#define ASR_FC_MAGIC_1 ((uint8_t)'F')
#define ASR_FC_COMMAND_PAYLOAD_SIZE 16u
#define ASR_FC_TELEMETRY_PAYLOAD_SIZE 28u

static void put_u16(uint8_t *out, uint16_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *out, uint32_t value) {
    out[0] = (uint8_t)value;
    out[1] = (uint8_t)(value >> 8);
    out[2] = (uint8_t)(value >> 16);
    out[3] = (uint8_t)(value >> 24);
}

static uint16_t get_u16(const uint8_t *in) {
    return (uint16_t)((uint16_t)in[0] | ((uint16_t)in[1] << 8));
}

static uint32_t get_u32(const uint8_t *in) {
    return (uint32_t)in[0] | ((uint32_t)in[1] << 8) |
        ((uint32_t)in[2] << 16) | ((uint32_t)in[3] << 24);
}

void asr_fc_motor_link_parser_init(asr_fc_motor_link_parser_t *parser) {
    if (parser) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void parser_resync(asr_fc_motor_link_parser_t *parser, uint8_t byte) {
    asr_fc_motor_link_parser_init(parser);
    if (byte == ASR_FC_MAGIC_0) {
        parser->buffer[0] = byte;
        parser->size = 1u;
    }
}

asr_fc_motor_link_status_t asr_fc_motor_link_parser_push(
    asr_fc_motor_link_parser_t *parser,
    uint8_t byte,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size,
    bool *frame_ready) {
    if (!parser || !frame || !frame_size || !frame_ready) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    *frame_ready = false;
    *frame_size = 0u;

    if (parser->size == 0u) {
        parser_resync(parser, byte);
        return ASR_FC_MOTOR_LINK_OK;
    }
    if (parser->size == 1u) {
        if (byte != ASR_FC_MAGIC_1) {
            parser_resync(parser, byte);
            return ASR_FC_MOTOR_LINK_OK;
        }
        parser->buffer[parser->size++] = byte;
        return ASR_FC_MOTOR_LINK_OK;
    }
    if (parser->size >= sizeof(parser->buffer)) {
        parser_resync(parser, byte);
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    parser->buffer[parser->size++] = byte;

    if (parser->size == ASR_FC_MOTOR_LINK_HEADER_SIZE) {
        const uint16_t payload_size = get_u16(&parser->buffer[4]);
        const uint8_t type = parser->buffer[3];
        if (parser->buffer[2] != ASR_FC_MOTOR_LINK_VERSION) {
            parser_resync(parser, byte);
            return ASR_FC_MOTOR_LINK_VERSION_MISMATCH;
        }
        if (payload_size > ASR_FC_MOTOR_LINK_MAX_PAYLOAD ||
            type < ASR_FC_MOTOR_LINK_COMMAND ||
            type > ASR_FC_MOTOR_LINK_HEARTBEAT) {
            parser_resync(parser, byte);
            return ASR_FC_MOTOR_LINK_INVALID_FRAME;
        }
        parser->expected_size = ASR_FC_MOTOR_LINK_HEADER_SIZE +
            payload_size + ASR_FC_MOTOR_LINK_CRC_SIZE;
    }
    if (parser->expected_size == 0u ||
        parser->size < parser->expected_size) {
        return ASR_FC_MOTOR_LINK_OK;
    }

    const size_t complete_size = parser->expected_size;
    const uint16_t expected_crc = get_u16(
        &parser->buffer[complete_size - ASR_FC_MOTOR_LINK_CRC_SIZE]);
    if (asr_fc_motor_link_crc16(parser->buffer,
            complete_size - ASR_FC_MOTOR_LINK_CRC_SIZE) != expected_crc) {
        parser_resync(parser, byte);
        return ASR_FC_MOTOR_LINK_CRC_FAILURE;
    }
    if (capacity < complete_size) {
        asr_fc_motor_link_parser_init(parser);
        return ASR_FC_MOTOR_LINK_BUFFER_TOO_SMALL;
    }
    memcpy(frame, parser->buffer, complete_size);
    *frame_size = complete_size;
    *frame_ready = true;
    asr_fc_motor_link_parser_init(parser);
    return ASR_FC_MOTOR_LINK_OK;
}

uint16_t asr_fc_motor_link_crc16(const uint8_t *data, size_t size) {
    if (!data && size != 0u) {
        return 0u;
    }
    uint16_t crc = 0xffffu;
    for (size_t index = 0; index < size; ++index) {
        crc ^= (uint16_t)data[index] << 8;
        for (uint32_t bit = 0; bit < 8u; ++bit) {
            crc = (crc & 0x8000u) != 0u
                ? (uint16_t)((crc << 1) ^ 0x1021u)
                : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static asr_fc_motor_link_status_t encode_header(
    asr_fc_motor_link_type_t type,
    uint16_t payload_size,
    uint32_t sequence,
    uint8_t *frame,
    size_t capacity) {
    if (!frame) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    const size_t required = ASR_FC_MOTOR_LINK_HEADER_SIZE + payload_size +
        ASR_FC_MOTOR_LINK_CRC_SIZE;
    if (capacity < required) {
        return ASR_FC_MOTOR_LINK_BUFFER_TOO_SMALL;
    }
    frame[0] = ASR_FC_MAGIC_0;
    frame[1] = ASR_FC_MAGIC_1;
    frame[2] = ASR_FC_MOTOR_LINK_VERSION;
    frame[3] = (uint8_t)type;
    put_u16(&frame[4], payload_size);
    put_u32(&frame[6], sequence);
    return ASR_FC_MOTOR_LINK_OK;
}

static asr_fc_motor_link_status_t decode_header(
    const uint8_t *frame,
    size_t frame_size,
    asr_fc_motor_link_type_t expected_type,
    uint16_t expected_payload_size,
    uint32_t *sequence) {
    if (!frame || !sequence) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    const size_t expected_size = ASR_FC_MOTOR_LINK_HEADER_SIZE +
        expected_payload_size + ASR_FC_MOTOR_LINK_CRC_SIZE;
    if (frame_size != expected_size || frame[0] != ASR_FC_MAGIC_0 ||
        frame[1] != ASR_FC_MAGIC_1 || frame[3] != (uint8_t)expected_type ||
        get_u16(&frame[4]) != expected_payload_size) {
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    if (frame[2] != ASR_FC_MOTOR_LINK_VERSION) {
        return ASR_FC_MOTOR_LINK_VERSION_MISMATCH;
    }
    const uint16_t expected_crc = get_u16(&frame[frame_size - 2u]);
    if (asr_fc_motor_link_crc16(frame, frame_size - 2u) != expected_crc) {
        return ASR_FC_MOTOR_LINK_CRC_FAILURE;
    }
    *sequence = get_u32(&frame[6]);
    return ASR_FC_MOTOR_LINK_OK;
}

static void finish_frame(uint8_t *frame, size_t size) {
    put_u16(&frame[size - 2u], asr_fc_motor_link_crc16(frame, size - 2u));
}

asr_fc_motor_link_status_t asr_fc_motor_link_encode_command(
    uint32_t sequence,
    const asr_fc_motor_command_t *command,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (!command || !frame_size) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    if (command->throttle_q15 > 32767u || command->motor_index >= 4u ||
        command->command_timeout_ms == 0u ||
        command->heartbeat_timeout_ms == 0u) {
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    asr_fc_motor_link_status_t status = encode_header(
        ASR_FC_MOTOR_LINK_COMMAND, ASR_FC_COMMAND_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != ASR_FC_MOTOR_LINK_OK) {
        return status;
    }
    uint8_t *payload = &frame[ASR_FC_MOTOR_LINK_HEADER_SIZE];
    memset(payload, 0, ASR_FC_COMMAND_PAYLOAD_SIZE);
    payload[0] = command->motor_index;
    payload[1] = command->armed ? 1u : 0u;
    put_u16(&payload[2], command->flags);
    put_u16(&payload[4], command->throttle_q15);
    put_u32(&payload[8], command->session_id);
    put_u16(&payload[12], command->command_timeout_ms);
    put_u16(&payload[14], command->heartbeat_timeout_ms);
    *frame_size = ASR_FC_MOTOR_LINK_HEADER_SIZE + ASR_FC_COMMAND_PAYLOAD_SIZE +
        ASR_FC_MOTOR_LINK_CRC_SIZE;
    finish_frame(frame, *frame_size);
    return ASR_FC_MOTOR_LINK_OK;
}

asr_fc_motor_link_status_t asr_fc_motor_link_decode_command(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_motor_command_t *command) {
    if (!command) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    memset(command, 0, sizeof(*command));
    asr_fc_motor_link_status_t status = decode_header(
        frame, frame_size, ASR_FC_MOTOR_LINK_COMMAND,
        ASR_FC_COMMAND_PAYLOAD_SIZE, sequence);
    if (status != ASR_FC_MOTOR_LINK_OK) {
        return status;
    }
    const uint8_t *payload = &frame[ASR_FC_MOTOR_LINK_HEADER_SIZE];
    if (payload[1] > 1u || payload[0] >= 4u || get_u16(&payload[4]) > 32767u ||
        get_u16(&payload[12]) == 0u || get_u16(&payload[14]) == 0u) {
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    command->motor_index = payload[0];
    command->armed = payload[1] != 0u;
    command->flags = get_u16(&payload[2]);
    command->throttle_q15 = get_u16(&payload[4]);
    command->session_id = get_u32(&payload[8]);
    command->command_timeout_ms = get_u16(&payload[12]);
    command->heartbeat_timeout_ms = get_u16(&payload[14]);
    return ASR_FC_MOTOR_LINK_OK;
}

asr_fc_motor_link_status_t asr_fc_motor_link_encode_telemetry(
    uint32_t sequence,
    const asr_fc_motor_telemetry_t *telemetry,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (!telemetry || !frame_size) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    if (telemetry->motor_index >= 4u || telemetry->duty_q15 > 32767u) {
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    asr_fc_motor_link_status_t status = encode_header(
        ASR_FC_MOTOR_LINK_TELEMETRY, ASR_FC_TELEMETRY_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != ASR_FC_MOTOR_LINK_OK) {
        return status;
    }
    uint8_t *payload = &frame[ASR_FC_MOTOR_LINK_HEADER_SIZE];
    memset(payload, 0, ASR_FC_TELEMETRY_PAYLOAD_SIZE);
    payload[0] = telemetry->motor_index;
    payload[1] = telemetry->motor_mode;
    put_u32(&payload[4], telemetry->fault_flags);
    put_u32(&payload[8], telemetry->bus_voltage_mv);
    put_u32(&payload[12], (uint32_t)telemetry->dc_current_ma);
    put_u32(&payload[16], telemetry->electrical_period_us);
    put_u16(&payload[20], telemetry->duty_q15);
    put_u32(&payload[24], telemetry->acknowledged_sequence);
    *frame_size = ASR_FC_MOTOR_LINK_HEADER_SIZE + ASR_FC_TELEMETRY_PAYLOAD_SIZE +
        ASR_FC_MOTOR_LINK_CRC_SIZE;
    finish_frame(frame, *frame_size);
    return ASR_FC_MOTOR_LINK_OK;
}

asr_fc_motor_link_status_t asr_fc_motor_link_decode_telemetry(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_motor_telemetry_t *telemetry) {
    if (!telemetry) {
        return ASR_FC_MOTOR_LINK_NULL_POINTER;
    }
    memset(telemetry, 0, sizeof(*telemetry));
    asr_fc_motor_link_status_t status = decode_header(
        frame, frame_size, ASR_FC_MOTOR_LINK_TELEMETRY,
        ASR_FC_TELEMETRY_PAYLOAD_SIZE, sequence);
    if (status != ASR_FC_MOTOR_LINK_OK) {
        return status;
    }
    const uint8_t *payload = &frame[ASR_FC_MOTOR_LINK_HEADER_SIZE];
    telemetry->motor_index = payload[0];
    telemetry->motor_mode = payload[1];
    telemetry->fault_flags = get_u32(&payload[4]);
    telemetry->bus_voltage_mv = get_u32(&payload[8]);
    telemetry->dc_current_ma = (int32_t)get_u32(&payload[12]);
    telemetry->electrical_period_us = get_u32(&payload[16]);
    telemetry->duty_q15 = get_u16(&payload[20]);
    telemetry->acknowledged_sequence = get_u32(&payload[24]);
    if (telemetry->motor_index >= 4u || telemetry->duty_q15 > 32767u) {
        memset(telemetry, 0, sizeof(*telemetry));
        return ASR_FC_MOTOR_LINK_INVALID_FRAME;
    }
    return ASR_FC_MOTOR_LINK_OK;
}
