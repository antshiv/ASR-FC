#ifndef ASR_FC_PROTOCOL_HIL_LINK_H
#define ASR_FC_PROTOCOL_HIL_LINK_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define ASR_FC_HIL_VERSION 1u
#define ASR_FC_HIL_HEADER_SIZE 10u
#define ASR_FC_HIL_CRC_SIZE 2u
#define ASR_FC_HIL_MAX_PAYLOAD 128u
#define ASR_FC_HIL_MAX_FRAME_SIZE \
    (ASR_FC_HIL_HEADER_SIZE + ASR_FC_HIL_MAX_PAYLOAD + ASR_FC_HIL_CRC_SIZE)

typedef enum {
    ASR_FC_HIL_SENSOR_GUIDANCE = 1,
    ASR_FC_HIL_FLIGHT_OUTPUT = 2
} asr_fc_hil_type_t;

typedef enum {
    ASR_FC_HIL_OK = 0,
    ASR_FC_HIL_NULL_POINTER,
    ASR_FC_HIL_BUFFER_TOO_SMALL,
    ASR_FC_HIL_INVALID_FRAME,
    ASR_FC_HIL_CRC_FAILURE,
    ASR_FC_HIL_VERSION_MISMATCH
} asr_fc_hil_status_t;

typedef struct {
    uint32_t session_id;
    uint64_t host_timestamp_us;
    uint64_t sensor_timestamp_us;
    bool arm_requested;
    uint8_t sensor_accuracy;
    float quaternion[4];
    float angular_rate[3];
    float linear_acceleration[3];
    float guidance_quaternion[4];
    float guidance_angular_rate[3];
    float collective_thrust_n;
} asr_fc_hil_sensor_guidance_t;

typedef struct {
    uint32_t session_id;
    uint32_t acknowledged_sequence;
    uint64_t device_timestamp_us;
    uint32_t execution_time_us;
    uint32_t fault_flags;
    uint32_t active_aiding_mask;
    uint8_t step_result;
    uint8_t flight_state;
    uint16_t motor_q15[4];
    float motor_speed_rad_s[4];
    float body_torque_nm[3];
    float collective_thrust_n;
    float observed_quaternion[4];
    float observed_angular_rate[3];
    float observed_linear_acceleration[3];
} asr_fc_hil_flight_output_t;

typedef struct {
    uint8_t buffer[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t size;
    size_t expected_size;
} asr_fc_hil_parser_t;

void asr_fc_hil_parser_init(asr_fc_hil_parser_t *parser);

asr_fc_hil_status_t asr_fc_hil_parser_push(
    asr_fc_hil_parser_t *parser,
    uint8_t byte,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size,
    bool *frame_ready);

asr_fc_hil_status_t asr_fc_hil_encode_sensor_guidance(
    uint32_t sequence,
    const asr_fc_hil_sensor_guidance_t *input,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size);

asr_fc_hil_status_t asr_fc_hil_decode_sensor_guidance(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_hil_sensor_guidance_t *output);

asr_fc_hil_status_t asr_fc_hil_encode_flight_output(
    uint32_t sequence,
    const asr_fc_hil_flight_output_t *input,
    uint8_t *frame,
    size_t capacity,
    size_t *frame_size);

asr_fc_hil_status_t asr_fc_hil_decode_flight_output(
    const uint8_t *frame,
    size_t frame_size,
    uint32_t *sequence,
    asr_fc_hil_flight_output_t *output);

#endif
