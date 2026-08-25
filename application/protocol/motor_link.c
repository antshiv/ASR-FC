#include "afc/protocol/motor_link.h"

#include <string.h>

#define AFC_MAGIC_0 ((uint8_t)'A')
#define AFC_MAGIC_1 ((uint8_t)'F')
#define AFC_COMMAND_PAYLOAD_SIZE 16u
#define AFC_TELEMETRY_PAYLOAD_SIZE 28u

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

uint16_t afc_motor_link_crc16(const uint8_t *data, size_t size) {
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

static afc_motor_link_status_t encode_header(
    afc_motor_link_type_t type,
    uint16_t payload_size,
    uint32_t sequence,
    uint8_t *frame,
    size_t capacity) {
    if (!frame) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    const size_t required = AFC_MOTOR_LINK_HEADER_SIZE + payload_size +
        AFC_MOTOR_LINK_CRC_SIZE;
    if (capacity < required) {
        return AFC_MOTOR_LINK_BUFFER_TOO_SMALL;
    }
    frame[0] = AFC_MAGIC_0;
    frame[1] = AFC_MAGIC_1;
    frame[2] = AFC_MOTOR_LINK_VERSION;
    frame[3] = (uint8_t)type;
    put_u16(&frame[4], payload_size);
    put_u32(&frame[6], sequence);
    return AFC_MOTOR_LINK_OK;
}

static afc_motor_link_status_t decode_header(
    const uint8_t *frame,
    size_t frame_size,
    afc_motor_link_type_t expected_type,
    uint16_t expected_payload_size,
    uint32_t *sequence) {
    if (!frame || !sequence) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    const size_t expected_size = AFC_MOTOR_LINK_HEADER_SIZE +
        expected_payload_size + AFC_MOTOR_LINK_CRC_SIZE;
    if (frame_size != expected_size || frame[0] != AFC_MAGIC_0 ||
        frame[1] != AFC_MAGIC_1 || frame[3] != (uint8_t)expected_type ||
        get_u16(&frame[4]) != expected_payload_size) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    if (frame[2] != AFC_MOTOR_LINK_VERSION) {
        return AFC_MOTOR_LINK_VERSION_MISMATCH;
    }
    const uint16_t expected_crc = get_u16(&frame[frame_size - 2u]);
    if (afc_motor_link_crc16(frame, frame_size - 2u) != expected_crc) {
        return AFC_MOTOR_LINK_CRC_FAILURE;
    }
    *sequence = get_u32(&frame[6]);
    return AFC_MOTOR_LINK_OK;
}

static void finish_frame(uint8_t *frame, size_t size) {
    put_u16(&frame[size - 2u], afc_motor_link_crc16(frame, size - 2u));
}

afc_motor_link_status_t afc_motor_link_encode_command(
    uint32_t sequence,
    const afc_motor_command_t *command,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (!command || !frame_size) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    if (command->throttle_q15 > 32767u || command->motor_index >= 4u ||
        command->command_timeout_ms == 0u ||
        command->heartbeat_timeout_ms == 0u) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    afc_motor_link_status_t status = encode_header(
        AFC_MOTOR_LINK_COMMAND, AFC_COMMAND_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != AFC_MOTOR_LINK_OK) {
        return status;
    }
    uint8_t *payload = &frame[AFC_MOTOR_LINK_HEADER_SIZE];
    memset(payload, 0, AFC_COMMAND_PAYLOAD_SIZE);
    payload[0] = command->motor_index;
    payload[1] = command->armed ? 1u : 0u;
    put_u16(&payload[2], command->flags);
    put_u16(&payload[4], command->throttle_q15);
    put_u32(&payload[8], command->session_id);
    put_u16(&payload[12], command->command_timeout_ms);
    put_u16(&payload[14], command->heartbeat_timeout_ms);
    *frame_size = AFC_MOTOR_LINK_HEADER_SIZE + AFC_COMMAND_PAYLOAD_SIZE +
        AFC_MOTOR_LINK_CRC_SIZE;
    finish_frame(frame, *frame_size);
    return AFC_MOTOR_LINK_OK;
}

afc_motor_link_status_t afc_motor_link_decode_command(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    afc_motor_command_t *command) {
    if (!command) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    memset(command, 0, sizeof(*command));
    afc_motor_link_status_t status = decode_header(
        frame, frame_size, AFC_MOTOR_LINK_COMMAND,
        AFC_COMMAND_PAYLOAD_SIZE, sequence);
    if (status != AFC_MOTOR_LINK_OK) {
        return status;
    }
    const uint8_t *payload = &frame[AFC_MOTOR_LINK_HEADER_SIZE];
    if (payload[1] > 1u || payload[0] >= 4u || get_u16(&payload[4]) > 32767u ||
        get_u16(&payload[12]) == 0u || get_u16(&payload[14]) == 0u) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    command->motor_index = payload[0];
    command->armed = payload[1] != 0u;
    command->flags = get_u16(&payload[2]);
    command->throttle_q15 = get_u16(&payload[4]);
    command->session_id = get_u32(&payload[8]);
    command->command_timeout_ms = get_u16(&payload[12]);
    command->heartbeat_timeout_ms = get_u16(&payload[14]);
    return AFC_MOTOR_LINK_OK;
}

afc_motor_link_status_t afc_motor_link_encode_telemetry(
    uint32_t sequence,
    const afc_motor_telemetry_t *telemetry,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (!telemetry || !frame_size) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    if (telemetry->motor_index >= 4u || telemetry->duty_q15 > 32767u) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    afc_motor_link_status_t status = encode_header(
        AFC_MOTOR_LINK_TELEMETRY, AFC_TELEMETRY_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != AFC_MOTOR_LINK_OK) {
        return status;
    }
    uint8_t *payload = &frame[AFC_MOTOR_LINK_HEADER_SIZE];
    memset(payload, 0, AFC_TELEMETRY_PAYLOAD_SIZE);
    payload[0] = telemetry->motor_index;
    payload[1] = telemetry->motor_mode;
    put_u32(&payload[4], telemetry->fault_flags);
    put_u32(&payload[8], telemetry->bus_voltage_mv);
    put_u32(&payload[12], (uint32_t)telemetry->dc_current_ma);
    put_u32(&payload[16], telemetry->electrical_period_us);
    put_u16(&payload[20], telemetry->duty_q15);
    put_u32(&payload[24], telemetry->acknowledged_sequence);
    *frame_size = AFC_MOTOR_LINK_HEADER_SIZE + AFC_TELEMETRY_PAYLOAD_SIZE +
        AFC_MOTOR_LINK_CRC_SIZE;
    finish_frame(frame, *frame_size);
    return AFC_MOTOR_LINK_OK;
}

afc_motor_link_status_t afc_motor_link_decode_telemetry(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    afc_motor_telemetry_t *telemetry) {
    if (!telemetry) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    memset(telemetry, 0, sizeof(*telemetry));
    afc_motor_link_status_t status = decode_header(
        frame, frame_size, AFC_MOTOR_LINK_TELEMETRY,
        AFC_TELEMETRY_PAYLOAD_SIZE, sequence);
    if (status != AFC_MOTOR_LINK_OK) {
        return status;
    }
    const uint8_t *payload = &frame[AFC_MOTOR_LINK_HEADER_SIZE];
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
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    return AFC_MOTOR_LINK_OK;
}
