#include "asr_fc/hil/flight_endpoint.h"

#include <assert.h>
#include <stdio.h>
#include <string.h>

static asr_fc_flight_config_t config(void) {
    asr_fc_flight_config_t value = {
        .attitude_gains = {
            {.kp = 0.30, .ki = 0.01, .kd = 0.05,
             .integrator_limit = 0.20, .output_limit = 0.50},
            {.kp = 0.30, .ki = 0.01, .kd = 0.05,
             .integrator_limit = 0.20, .output_limit = 0.50},
            {.kp = 0.20, .ki = 0.00, .kd = 0.03,
             .integrator_limit = 0.00, .output_limit = 0.20},
        },
        .rate_weight = 1.0,
        .max_rotor_speed_rad_s = 1000.0,
        .min_dt_s = 0.005,
        .max_dt_s = 0.020,
        .sensor_timeout_us = 30000u,
        .minimum_ceva_accuracy = 1u,
    };
    const double positions[4][3] = {
        {0.18, 0.18, 0.0}, {-0.18, 0.18, 0.0},
        {-0.18, -0.18, 0.0}, {0.18, -0.18, 0.0},
    };
    const double directions[4] = {1.0, -1.0, 1.0, -1.0};
    for (size_t index = 0u; index < 4u; ++index) {
        memcpy(value.rotors[index].position, positions[index],
               sizeof(positions[index]));
        value.rotors[index].axis[2] = 1.0;
        value.rotors[index].direction = directions[index];
        value.rotors[index].thrust_coeff = 8.0e-6;
        value.rotors[index].torque_coeff = 1.2e-7;
    }
    return value;
}

static size_t request_frame(uint32_t session, uint32_t sequence,
                            uint64_t timestamp, bool armed,
                            uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE]) {
    const asr_fc_hil_sensor_guidance_t request = {
        .session_id = session,
        .host_timestamp_us = timestamp,
        .sensor_timestamp_us = timestamp,
        .arm_requested = armed,
        .sensor_accuracy = 3u,
        .quaternion = {0.9914449f, 0.1305262f, 0.0f, 0.0f},
        .angular_rate = {0.0f, 0.0f, 0.0f},
        .linear_acceleration = {0.0f, 0.0f, 9.80665f},
        .guidance_quaternion = {1.0f, 0.0f, 0.0f, 0.0f},
        .guidance_angular_rate = {0.0f, 0.0f, 0.0f},
        .collective_thrust_n = 11.76798f,
    };
    size_t size = 0u;
    assert(asr_fc_hil_encode_sensor_guidance(
        sequence, &request, frame, ASR_FC_HIL_MAX_FRAME_SIZE,
        &size) == ASR_FC_HIL_OK);
    return size;
}

static void test_lockstep_and_session_reset(void) {
    asr_fc_hil_flight_endpoint_t endpoint;
    const asr_fc_flight_config_t cfg = config();
    assert(asr_fc_hil_flight_endpoint_init(&endpoint, &cfg) ==
           ASR_FC_STEP_OK);

    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t size = request_frame(1u, 1u, 10000u, true, frame);
    uint32_t sequence = 0u;
    asr_fc_hil_flight_output_t response;
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 500u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(response.step_result == ASR_FC_STEP_OK);
    assert(response.flight_state == ASR_FC_FLIGHT_ARMED);
    assert(response.acknowledged_sequence == 1u);
    assert(response.motor_q15[0] > 0u);

    size = request_frame(1u, 2u, 20000u, true, frame);
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 600u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(response.step_result == ASR_FC_STEP_OK);

    size = request_frame(2u, 1u, 10000u, false, frame);
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 700u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(response.step_result == ASR_FC_STEP_OK);
    assert(response.flight_state == ASR_FC_FLIGHT_DISARMED);
}

static void test_duplicate_and_corruption_fail_closed(void) {
    asr_fc_hil_flight_endpoint_t endpoint;
    const asr_fc_flight_config_t cfg = config();
    assert(asr_fc_hil_flight_endpoint_init(&endpoint, &cfg) ==
           ASR_FC_STEP_OK);

    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    const size_t size = request_frame(7u, 1u, 10000u, true, frame);
    uint32_t sequence = 0u;
    asr_fc_hil_flight_output_t response;
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 500u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 600u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(response.step_result == ASR_FC_STEP_SENSOR_INVALID);
    assert(response.flight_state == ASR_FC_FLIGHT_FAILSAFE);

    frame[20] ^= 1u;
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 700u, &sequence, &response) ==
        ASR_FC_HIL_CRC_FAILURE);
    assert(endpoint.flight_core.state == ASR_FC_FLIGHT_DISARMED);
}

static void test_wire_magnitude_maps_to_negative_body_axis(void) {
    asr_fc_hil_flight_endpoint_t endpoint;
    asr_fc_flight_config_t cfg = config();
    for (size_t index = 0u; index < 4u; ++index) {
        cfg.rotors[index].axis[2] = -1.0;
    }
    assert(asr_fc_hil_flight_endpoint_init(&endpoint, &cfg) ==
           ASR_FC_STEP_OK);

    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    const size_t size = request_frame(9u, 1u, 10000u, true, frame);
    uint32_t sequence = 0u;
    asr_fc_hil_flight_output_t response;
    assert(asr_fc_hil_flight_endpoint_step(
        &endpoint, frame, size, 500u, &sequence, &response) ==
        ASR_FC_HIL_OK);
    assert(response.step_result == ASR_FC_STEP_OK);
    assert(endpoint.flight_core.state == ASR_FC_FLIGHT_ARMED);
    assert(endpoint.flight_core.config.rotors[0].axis[2] == -1.0);
    assert(response.collective_thrust_n == 11.76798f);
    assert(response.motor_q15[0] > 0u);
}

int main(void) {
    test_lockstep_and_session_reset();
    test_duplicate_and_corruption_fail_closed();
    test_wire_magnitude_maps_to_negative_body_axis();
    puts("ASR-FC HIL flight endpoint tests passed");
    return 0;
}
