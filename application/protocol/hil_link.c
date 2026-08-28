#include "asr_fc/protocol/hil_link.h"

#include "asr_fc/protocol/motor_link.h"

#include <math.h>
#include <string.h>

#define HIL_MAGIC_0 ((uint8_t)'A')
#define HIL_MAGIC_1 ((uint8_t)'H')
#define SENSOR_GUIDANCE_PAYLOAD_SIZE 96u
#define FLIGHT_OUTPUT_PAYLOAD_SIZE 112u

_Static_assert(sizeof(float) == 4u, "HIL protocol requires IEEE-754 binary32");

static void put_u16(uint8_t *output, uint16_t value) {
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
}

static void put_u32(uint8_t *output, uint32_t value) {
    output[0] = (uint8_t)value;
    output[1] = (uint8_t)(value >> 8);
    output[2] = (uint8_t)(value >> 16);
    output[3] = (uint8_t)(value >> 24);
}

static void put_u64(uint8_t *output, uint64_t value) {
    put_u32(output, (uint32_t)value);
    put_u32(output + 4u, (uint32_t)(value >> 32));
}

static uint16_t get_u16(const uint8_t *input) {
    return (uint16_t)((uint16_t)input[0] | ((uint16_t)input[1] << 8));
}

static uint32_t get_u32(const uint8_t *input) {
    return (uint32_t)input[0] | ((uint32_t)input[1] << 8) |
        ((uint32_t)input[2] << 16) | ((uint32_t)input[3] << 24);
}

static uint64_t get_u64(const uint8_t *input) {
    return (uint64_t)get_u32(input) | ((uint64_t)get_u32(input + 4u) << 32);
}

static void put_f32(uint8_t *output, float value) {
    uint32_t bits = 0u;
    memcpy(&bits, &value, sizeof(bits));
    put_u32(output, bits);
}

static float get_f32(const uint8_t *input) {
    const uint32_t bits = get_u32(input);
    float value = 0.0f;
    memcpy(&value, &bits, sizeof(value));
    return value;
}

static bool finite_values(const float *values, size_t count) {
    for (size_t index = 0u; index < count; ++index) {
        if (!isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

static bool valid_quaternion(const float values[4]) {
    if (!finite_values(values, 4u)) {
        return false;
    }
    float norm_squared = 0.0f;
    for (size_t index = 0u; index < 4u; ++index) {
        norm_squared += values[index] * values[index];
    }
    return norm_squared > 0.81f && norm_squared < 1.21f;
}

static void put_vector(uint8_t *output, const float *values, size_t count) {
    for (size_t index = 0u; index < count; ++index) {
        put_f32(output + index * 4u, values[index]);
    }
}

static void get_vector(const uint8_t *input, float *values, size_t count) {
    for (size_t index = 0u; index < count; ++index) {
        values[index] = get_f32(input + index * 4u);
    }
}

void asr_fc_hil_parser_init(asr_fc_hil_parser_t *parser) {
    if (parser != NULL) {
        memset(parser, 0, sizeof(*parser));
    }
}

static void parser_resync(asr_fc_hil_parser_t *parser, uint8_t byte) {
    asr_fc_hil_parser_init(parser);
    if (byte == HIL_MAGIC_0) {
        parser->buffer[0] = byte;
        parser->size = 1u;
    }
}

asr_fc_hil_status_t asr_fc_hil_parser_push(
    asr_fc_hil_parser_t *parser,
    uint8_t byte,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size,
    bool *frame_ready) {
    if (parser == NULL || frame == NULL || frame_size == NULL ||
        frame_ready == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    *frame_size = 0u;
    *frame_ready = false;
    if (parser->size == 0u) {
        parser_resync(parser, byte);
        return ASR_FC_HIL_OK;
    }
    if (parser->size == 1u) {
        if (byte != HIL_MAGIC_1) {
            parser_resync(parser, byte);
            return ASR_FC_HIL_OK;
        }
        parser->buffer[parser->size++] = byte;
        return ASR_FC_HIL_OK;
    }
    if (parser->size >= sizeof(parser->buffer)) {
        parser_resync(parser, byte);
        return ASR_FC_HIL_INVALID_FRAME;
    }
    parser->buffer[parser->size++] = byte;
    if (parser->size == ASR_FC_HIL_HEADER_SIZE) {
        const uint16_t payload_size = get_u16(&parser->buffer[4]);
        const uint8_t type = parser->buffer[3];
        if (parser->buffer[2] != ASR_FC_HIL_VERSION) {
            parser_resync(parser, byte);
            return ASR_FC_HIL_VERSION_MISMATCH;
        }
        if (payload_size > ASR_FC_HIL_MAX_PAYLOAD ||
            type < ASR_FC_HIL_SENSOR_GUIDANCE ||
            type > ASR_FC_HIL_FLIGHT_OUTPUT) {
            parser_resync(parser, byte);
            return ASR_FC_HIL_INVALID_FRAME;
        }
        parser->expected_size = ASR_FC_HIL_HEADER_SIZE + payload_size +
            ASR_FC_HIL_CRC_SIZE;
    }
    if (parser->expected_size == 0u || parser->size < parser->expected_size) {
        return ASR_FC_HIL_OK;
    }
    const size_t complete_size = parser->expected_size;
    const uint16_t expected_crc = get_u16(&parser->buffer[complete_size - 2u]);
    if (asr_fc_motor_link_crc16(parser->buffer, complete_size - 2u) !=
        expected_crc) {
        parser_resync(parser, byte);
        return ASR_FC_HIL_CRC_FAILURE;
    }
    if (capacity < complete_size) {
        asr_fc_hil_parser_init(parser);
        return ASR_FC_HIL_BUFFER_TOO_SMALL;
    }
    memcpy(frame, parser->buffer, complete_size);
    *frame_size = complete_size;
    *frame_ready = true;
    asr_fc_hil_parser_init(parser);
    return ASR_FC_HIL_OK;
}

static asr_fc_hil_status_t encode_header(asr_fc_hil_type_t type,
                                         uint16_t payload_size,
                                         uint32_t sequence,
                                         uint8_t *frame,
                                         size_t capacity) {
    const size_t required = ASR_FC_HIL_HEADER_SIZE + payload_size +
        ASR_FC_HIL_CRC_SIZE;
    if (frame == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    if (capacity < required) {
        return ASR_FC_HIL_BUFFER_TOO_SMALL;
    }
    frame[0] = HIL_MAGIC_0;
    frame[1] = HIL_MAGIC_1;
    frame[2] = ASR_FC_HIL_VERSION;
    frame[3] = (uint8_t)type;
    put_u16(&frame[4], payload_size);
    put_u32(&frame[6], sequence);
    return ASR_FC_HIL_OK;
}

static asr_fc_hil_status_t decode_header(const uint8_t *frame,
                                         size_t frame_size,
                                         asr_fc_hil_type_t type,
                                         uint16_t payload_size,
                                         uint32_t *sequence) {
    const size_t expected_size = ASR_FC_HIL_HEADER_SIZE + payload_size +
        ASR_FC_HIL_CRC_SIZE;
    if (frame == NULL || sequence == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    if (frame_size != expected_size || frame[0] != HIL_MAGIC_0 ||
        frame[1] != HIL_MAGIC_1 || frame[3] != (uint8_t)type ||
        get_u16(&frame[4]) != payload_size) {
        return ASR_FC_HIL_INVALID_FRAME;
    }
    if (frame[2] != ASR_FC_HIL_VERSION) {
        return ASR_FC_HIL_VERSION_MISMATCH;
    }
    if (asr_fc_motor_link_crc16(frame, frame_size - 2u) !=
        get_u16(&frame[frame_size - 2u])) {
        return ASR_FC_HIL_CRC_FAILURE;
    }
    *sequence = get_u32(&frame[6]);
    return ASR_FC_HIL_OK;
}

static void finish_frame(uint8_t *frame, size_t size) {
    put_u16(&frame[size - 2u], asr_fc_motor_link_crc16(frame, size - 2u));
}

asr_fc_hil_status_t asr_fc_hil_encode_sensor_guidance(
    uint32_t sequence,
    const asr_fc_hil_sensor_guidance_t *input,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (input == NULL || frame_size == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    if (input->session_id == 0u || input->sensor_timestamp_us == 0u ||
        input->sensor_accuracy > 3u ||
        !valid_quaternion(input->quaternion) ||
        !valid_quaternion(input->guidance_quaternion) ||
        !finite_values(input->angular_rate, 3u) ||
        !finite_values(input->linear_acceleration, 3u) ||
        !finite_values(input->guidance_angular_rate, 3u) ||
        !isfinite(input->collective_thrust_n) || input->collective_thrust_n < 0.0f) {
        return ASR_FC_HIL_INVALID_FRAME;
    }
    asr_fc_hil_status_t status = encode_header(
        ASR_FC_HIL_SENSOR_GUIDANCE, SENSOR_GUIDANCE_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != ASR_FC_HIL_OK) {
        return status;
    }
    uint8_t *payload = &frame[ASR_FC_HIL_HEADER_SIZE];
    memset(payload, 0, SENSOR_GUIDANCE_PAYLOAD_SIZE);
    put_u32(payload, input->session_id);
    put_u64(payload + 4u, input->host_timestamp_us);
    put_u64(payload + 12u, input->sensor_timestamp_us);
    payload[20] = input->arm_requested ? 1u : 0u;
    payload[21] = input->sensor_accuracy;
    put_vector(payload + 24u, input->quaternion, 4u);
    put_vector(payload + 40u, input->angular_rate, 3u);
    put_vector(payload + 52u, input->linear_acceleration, 3u);
    put_vector(payload + 64u, input->guidance_quaternion, 4u);
    put_vector(payload + 80u, input->guidance_angular_rate, 3u);
    put_f32(payload + 92u, input->collective_thrust_n);
    *frame_size = ASR_FC_HIL_HEADER_SIZE + SENSOR_GUIDANCE_PAYLOAD_SIZE + 2u;
    finish_frame(frame, *frame_size);
    return ASR_FC_HIL_OK;
}

asr_fc_hil_status_t asr_fc_hil_decode_sensor_guidance(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_hil_sensor_guidance_t *output) {
    if (output == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    memset(output, 0, sizeof(*output));
    asr_fc_hil_status_t status = decode_header(
        frame, frame_size, ASR_FC_HIL_SENSOR_GUIDANCE,
        SENSOR_GUIDANCE_PAYLOAD_SIZE, sequence);
    if (status != ASR_FC_HIL_OK) {
        return status;
    }
    const uint8_t *payload = &frame[ASR_FC_HIL_HEADER_SIZE];
    output->session_id = get_u32(payload);
    output->host_timestamp_us = get_u64(payload + 4u);
    output->sensor_timestamp_us = get_u64(payload + 12u);
    output->arm_requested = payload[20] != 0u;
    output->sensor_accuracy = payload[21];
    get_vector(payload + 24u, output->quaternion, 4u);
    get_vector(payload + 40u, output->angular_rate, 3u);
    get_vector(payload + 52u, output->linear_acceleration, 3u);
    get_vector(payload + 64u, output->guidance_quaternion, 4u);
    get_vector(payload + 80u, output->guidance_angular_rate, 3u);
    output->collective_thrust_n = get_f32(payload + 92u);
    if (payload[20] > 1u || output->session_id == 0u ||
        output->sensor_timestamp_us == 0u || output->sensor_accuracy > 3u ||
        !valid_quaternion(output->quaternion) ||
        !valid_quaternion(output->guidance_quaternion) ||
        !finite_values(output->angular_rate, 3u) ||
        !finite_values(output->linear_acceleration, 3u) ||
        !finite_values(output->guidance_angular_rate, 3u) ||
        !isfinite(output->collective_thrust_n) ||
        output->collective_thrust_n < 0.0f) {
        memset(output, 0, sizeof(*output));
        return ASR_FC_HIL_INVALID_FRAME;
    }
    return ASR_FC_HIL_OK;
}

asr_fc_hil_status_t asr_fc_hil_encode_flight_output(
    uint32_t sequence,
    const asr_fc_hil_flight_output_t *input,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size) {
    if (input == NULL || frame_size == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    if (input->session_id == 0u || input->acknowledged_sequence == 0u ||
        input->flight_state > 2u || input->step_result > 9u ||
        !finite_values(input->motor_speed_rad_s, 4u) ||
        !finite_values(input->body_torque_nm, 3u) ||
        !isfinite(input->collective_thrust_n) ||
        input->collective_thrust_n < 0.0f ||
        !valid_quaternion(input->observed_quaternion) ||
        !finite_values(input->observed_angular_rate, 3u) ||
        !finite_values(input->observed_linear_acceleration, 3u)) {
        return ASR_FC_HIL_INVALID_FRAME;
    }
    for (size_t index = 0u; index < 4u; ++index) {
        if (input->motor_q15[index] > 32767u ||
            input->motor_speed_rad_s[index] < 0.0f) {
            return ASR_FC_HIL_INVALID_FRAME;
        }
    }
    asr_fc_hil_status_t status = encode_header(
        ASR_FC_HIL_FLIGHT_OUTPUT, FLIGHT_OUTPUT_PAYLOAD_SIZE, sequence,
        frame, capacity);
    if (status != ASR_FC_HIL_OK) {
        return status;
    }
    uint8_t *payload = &frame[ASR_FC_HIL_HEADER_SIZE];
    memset(payload, 0, FLIGHT_OUTPUT_PAYLOAD_SIZE);
    put_u32(payload, input->session_id);
    put_u32(payload + 4u, input->acknowledged_sequence);
    put_u64(payload + 8u, input->device_timestamp_us);
    put_u32(payload + 16u, input->execution_time_us);
    put_u32(payload + 20u, input->fault_flags);
    put_u32(payload + 24u, input->active_aiding_mask);
    payload[28] = input->step_result;
    payload[29] = input->flight_state;
    for (size_t index = 0u; index < 4u; ++index) {
        put_u16(payload + 32u + index * 2u, input->motor_q15[index]);
    }
    put_vector(payload + 40u, input->motor_speed_rad_s, 4u);
    put_vector(payload + 56u, input->body_torque_nm, 3u);
    put_f32(payload + 68u, input->collective_thrust_n);
    put_vector(payload + 72u, input->observed_quaternion, 4u);
    put_vector(payload + 88u, input->observed_angular_rate, 3u);
    put_vector(payload + 100u, input->observed_linear_acceleration, 3u);
    *frame_size = ASR_FC_HIL_HEADER_SIZE + FLIGHT_OUTPUT_PAYLOAD_SIZE + 2u;
    finish_frame(frame, *frame_size);
    return ASR_FC_HIL_OK;
}

asr_fc_hil_status_t asr_fc_hil_decode_flight_output(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_hil_flight_output_t *output) {
    if (output == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    memset(output, 0, sizeof(*output));
    asr_fc_hil_status_t status = decode_header(
        frame, frame_size, ASR_FC_HIL_FLIGHT_OUTPUT,
        FLIGHT_OUTPUT_PAYLOAD_SIZE, sequence);
    if (status != ASR_FC_HIL_OK) {
        return status;
    }
    const uint8_t *payload = &frame[ASR_FC_HIL_HEADER_SIZE];
    output->session_id = get_u32(payload);
    output->acknowledged_sequence = get_u32(payload + 4u);
    output->device_timestamp_us = get_u64(payload + 8u);
    output->execution_time_us = get_u32(payload + 16u);
    output->fault_flags = get_u32(payload + 20u);
    output->active_aiding_mask = get_u32(payload + 24u);
    output->step_result = payload[28];
    output->flight_state = payload[29];
    for (size_t index = 0u; index < 4u; ++index) {
        output->motor_q15[index] = get_u16(payload + 32u + index * 2u);
    }
    get_vector(payload + 40u, output->motor_speed_rad_s, 4u);
    get_vector(payload + 56u, output->body_torque_nm, 3u);
    output->collective_thrust_n = get_f32(payload + 68u);
    get_vector(payload + 72u, output->observed_quaternion, 4u);
    get_vector(payload + 88u, output->observed_angular_rate, 3u);
    get_vector(payload + 100u, output->observed_linear_acceleration, 3u);
    if (output->session_id == 0u || output->acknowledged_sequence == 0u ||
        output->flight_state > 2u || output->step_result > 9u ||
        !finite_values(output->motor_speed_rad_s, 4u) ||
        !finite_values(output->body_torque_nm, 3u) ||
        !isfinite(output->collective_thrust_n) ||
        output->collective_thrust_n < 0.0f ||
        !valid_quaternion(output->observed_quaternion) ||
        !finite_values(output->observed_angular_rate, 3u) ||
        !finite_values(output->observed_linear_acceleration, 3u)) {
        memset(output, 0, sizeof(*output));
        return ASR_FC_HIL_INVALID_FRAME;
    }
    for (size_t index = 0u; index < 4u; ++index) {
        if (output->motor_q15[index] > 32767u ||
            output->motor_speed_rad_s[index] < 0.0f) {
            memset(output, 0, sizeof(*output));
            return ASR_FC_HIL_INVALID_FRAME;
        }
    }
    return ASR_FC_HIL_OK;
}
