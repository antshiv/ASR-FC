#include "asr_fc/protocol/hil_link.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static asr_fc_hil_sensor_guidance_t request_fixture(void) {
    asr_fc_hil_sensor_guidance_t value = {
        .session_id = 0x41535231u,
        .host_timestamp_us = 1000000u,
        .sensor_timestamp_us = 990000u,
        .arm_requested = true,
        .sensor_accuracy = 3u,
        .quaternion = {0.9914449f, 0.1305262f, 0.0f, 0.0f},
        .angular_rate = {0.1f, -0.2f, 0.3f},
        .linear_acceleration = {0.0f, 0.0f, 9.81f},
        .guidance_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .guidance_angular_rate = {0.0f, 0.0f, 0.0f},
        .collective_thrust_n = 11.76798f,
    };
    return value;
}

static asr_fc_hil_flight_output_t response_fixture(void) {
    asr_fc_hil_flight_output_t value = {
        .session_id = 0x41535231u,
        .acknowledged_sequence = 42u,
        .device_timestamp_us = 1000100u,
        .execution_time_us = 571u,
        .fault_flags = 0u,
        .active_aiding_mask = 0u,
        .step_result = 0u,
        .flight_state = 1u,
        .motor_q15 = {10000u, 11000u, 12000u, 13000u},
        .motor_speed_rad_s = {305.2f, 335.7f, 366.2f, 396.7f},
        .body_torque_nm = {0.1f, -0.2f, 0.03f},
        .collective_thrust_n = 11.76798f,
        .observed_quaternion = {0.9914449f, 0.1305262f, 0.0f, 0.0f},
        .observed_angular_rate = {0.1f, -0.2f, 0.3f},
        .observed_linear_acceleration = {0.0f, 0.0f, 9.81f},
    };
    return value;
}

static void test_request_round_trip(void) {
    const asr_fc_hil_sensor_guidance_t input = request_fixture();
    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t frame_size = 0u;
    assert(asr_fc_hil_encode_sensor_guidance(
        42u, &input, frame, sizeof(frame), &frame_size) == ASR_FC_HIL_OK);
    assert(frame_size == 108u);
    uint32_t sequence = 0u;
    asr_fc_hil_sensor_guidance_t output;
    assert(asr_fc_hil_decode_sensor_guidance(
        frame, frame_size, &sequence, &output) == ASR_FC_HIL_OK);
    assert(sequence == 42u);
    assert(output.session_id == input.session_id);
    assert(output.sensor_timestamp_us == input.sensor_timestamp_us);
    assert(output.arm_requested);
    assert(memcmp(output.quaternion, input.quaternion,
                  sizeof(input.quaternion)) == 0);
    assert(output.collective_thrust_n == input.collective_thrust_n);
}

static void test_response_round_trip(void) {
    const asr_fc_hil_flight_output_t input = response_fixture();
    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t frame_size = 0u;
    assert(asr_fc_hil_encode_flight_output(
        43u, &input, frame, sizeof(frame), &frame_size) == ASR_FC_HIL_OK);
    assert(frame_size == 124u);
    uint32_t sequence = 0u;
    asr_fc_hil_flight_output_t output;
    assert(asr_fc_hil_decode_flight_output(
        frame, frame_size, &sequence, &output) == ASR_FC_HIL_OK);
    assert(sequence == 43u);
    assert(output.acknowledged_sequence == 42u);
    assert(output.execution_time_us == input.execution_time_us);
    assert(output.motor_q15[3] == input.motor_q15[3]);
    assert(output.motor_speed_rad_s[2] == input.motor_speed_rad_s[2]);
}

static void test_corruption_and_invalid_values_fail_closed(void) {
    asr_fc_hil_sensor_guidance_t input = request_fixture();
    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t frame_size = 0u;
    assert(asr_fc_hil_encode_sensor_guidance(
        1u, &input, frame, sizeof(frame), &frame_size) == ASR_FC_HIL_OK);
    frame[40] ^= 0x01u;
    uint32_t sequence = 0u;
    asr_fc_hil_sensor_guidance_t output;
    memset(&output, 0xff, sizeof(output));
    assert(asr_fc_hil_decode_sensor_guidance(
        frame, frame_size, &sequence, &output) == ASR_FC_HIL_CRC_FAILURE);
    assert(output.session_id == 0u);
    input.angular_rate[1] = NAN;
    assert(asr_fc_hil_encode_sensor_guidance(
        2u, &input, frame, sizeof(frame), &frame_size) ==
        ASR_FC_HIL_INVALID_FRAME);

    asr_fc_hil_flight_output_t response = response_fixture();
    response.collective_thrust_n = -1.0f;
    assert(asr_fc_hil_encode_flight_output(
        3u, &response, frame, sizeof(frame), &frame_size) ==
        ASR_FC_HIL_INVALID_FRAME);
}

static void test_stream_parser_resynchronizes(void) {
    const asr_fc_hil_sensor_guidance_t input = request_fixture();
    uint8_t encoded[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t encoded_size = 0u;
    assert(asr_fc_hil_encode_sensor_guidance(
        8u, &input, encoded, sizeof(encoded), &encoded_size) == ASR_FC_HIL_OK);
    asr_fc_hil_parser_t parser;
    asr_fc_hil_parser_init(&parser);
    uint8_t decoded[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t decoded_size = 0u;
    bool ready = false;
    const uint8_t noise[] = {0xffu, (uint8_t)'A', 0x00u, (uint8_t)'A'};
    for (size_t index = 0u; index < sizeof(noise); ++index) {
        assert(asr_fc_hil_parser_push(&parser, noise[index], decoded,
            sizeof(decoded), &decoded_size, &ready) == ASR_FC_HIL_OK);
        assert(!ready);
    }
    for (size_t index = 1u; index < encoded_size; ++index) {
        assert(asr_fc_hil_parser_push(&parser, encoded[index], decoded,
            sizeof(decoded), &decoded_size, &ready) == ASR_FC_HIL_OK);
    }
    assert(ready);
    assert(decoded_size == encoded_size);
}

int main(void) {
    test_request_round_trip();
    test_response_round_trip();
    test_corruption_and_invalid_values_fail_closed();
    test_stream_parser_resynchronizes();
    puts("ASR-FC host HIL-link tests passed");
    return 0;
}
