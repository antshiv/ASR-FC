#include "asr_fc/flight/flight_core.h"
#include "asr_fc/protocol/hil_link.h"

#include <string.h>

#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/kernel.h>
#include <zephyr/timing/timing.h>

enum { ASR_FC_PHYSICAL_OUTPUTS_ENABLED = 0 };
BUILD_ASSERT(ASR_FC_PHYSICAL_OUTPUTS_ENABLED == 0,
             "Host HIL must not drive physical outputs");

static const struct device *const hil_uart = DEVICE_DT_GET(DT_NODELABEL(uart0));
static asr_fc_hil_parser_t parser;
static asr_fc_flight_core_t flight_core;
static uint32_t active_session;

static asr_fc_flight_config_t flight_config(void) {
    asr_fc_flight_config_t config = {
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
        memcpy(config.rotors[index].position, positions[index],
               sizeof(positions[index]));
        config.rotors[index].axis[2] = 1.0;
        config.rotors[index].direction = directions[index];
        config.rotors[index].thrust_coeff = 8.0e-6;
        config.rotors[index].torque_coeff = 1.2e-7;
    }
    return config;
}

static bool reset_flight_core(void) {
    const asr_fc_flight_config_t config = flight_config();
    return asr_fc_flight_init(&flight_core, &config) == ASR_FC_STEP_OK;
}

static void write_frame(const uint8_t *frame, size_t frame_size) {
    for (size_t index = 0u; index < frame_size; ++index) {
        uart_poll_out(hil_uart, frame[index]);
    }
}

static void copy_float_to_double(double *output, const float *input,
                                 size_t count) {
    for (size_t index = 0u; index < count; ++index) {
        output[index] = input[index];
    }
}

static void copy_double_to_float(float *output, const double *input,
                                 size_t count) {
    for (size_t index = 0u; index < count; ++index) {
        output[index] = (float)input[index];
    }
}

static void process_frame(const uint8_t *frame, size_t frame_size) {
    uint32_t sequence = 0u;
    asr_fc_hil_sensor_guidance_t request;
    if (asr_fc_hil_decode_sensor_guidance(
            frame, frame_size, &sequence, &request) != ASR_FC_HIL_OK) {
        asr_fc_flight_disarm(&flight_core);
        return;
    }
    if (request.session_id != active_session) {
        if (!reset_flight_core()) {
            return;
        }
        active_session = request.session_id;
    }

    asr_fc_ceva_sample_t sample = {
        .sequence = sequence,
        .timestamp_us = request.sensor_timestamp_us,
        .accuracy = request.sensor_accuracy,
        .attitude_valid = true,
    };
    copy_float_to_double(sample.quaternion, request.quaternion, 4u);
    copy_float_to_double(sample.angular_rate, request.angular_rate, 3u);
    copy_float_to_double(sample.linear_acceleration,
                         request.linear_acceleration, 3u);

    asr_fc_guidance_t guidance = {
        .collective_thrust_n = request.collective_thrust_n,
        .arm_requested = request.arm_requested,
    };
    copy_float_to_double(guidance.quaternion,
                         request.guidance_quaternion, 4u);
    copy_float_to_double(guidance.angular_rate,
                         request.guidance_angular_rate, 3u);

    asr_fc_flight_output_t output;
    timing_t started = timing_counter_get();
    const asr_fc_step_result_t step = asr_fc_flight_step(
        &flight_core, &sample, NULL, &guidance,
        request.sensor_timestamp_us, &output);
    timing_t finished = timing_counter_get();

    asr_fc_hil_flight_output_t response = {
        .session_id = request.session_id,
        .acknowledged_sequence = sequence,
        .device_timestamp_us = k_cyc_to_us_floor32(k_cycle_get_32()),
        .execution_time_us = (uint32_t)timing_cycles_to_ns(
            timing_cycles_get(&started, &finished)) / 1000u,
        .fault_flags = step == ASR_FC_STEP_OK ? 0u : (1u << (uint32_t)step),
        .active_aiding_mask = output.active_aiding_mask,
        .step_result = (uint8_t)step,
        .flight_state = (uint8_t)output.state,
        .collective_thrust_n = (float)output.actuator.collective_thrust,
    };
    memcpy(response.motor_q15, output.motor_q15,
           sizeof(response.motor_q15));
    copy_double_to_float(response.motor_speed_rad_s,
                         output.motor_speed_rad_s, 4u);
    copy_double_to_float(response.body_torque_nm,
                         output.actuator.body_torque, 3u);
    memcpy(response.observed_quaternion, request.quaternion,
           sizeof(response.observed_quaternion));
    memcpy(response.observed_angular_rate, request.angular_rate,
           sizeof(response.observed_angular_rate));
    memcpy(response.observed_linear_acceleration,
           request.linear_acceleration,
           sizeof(response.observed_linear_acceleration));

    uint8_t encoded[ASR_FC_HIL_MAX_FRAME_SIZE];
    size_t encoded_size = 0u;
    if (asr_fc_hil_encode_flight_output(
            sequence, &response, encoded, sizeof(encoded),
            &encoded_size) == ASR_FC_HIL_OK) {
        write_frame(encoded, encoded_size);
    } else {
        asr_fc_flight_disarm(&flight_core);
    }
}

int main(void) {
    if (!device_is_ready(hil_uart) || !reset_flight_core()) {
        return 1;
    }
    asr_fc_hil_parser_init(&parser);
    timing_init();
    timing_start();

    uint8_t frame[ASR_FC_HIL_MAX_FRAME_SIZE];
    for (;;) {
        uint8_t byte = 0u;
        if (uart_poll_in(hil_uart, &byte) != 0) {
            k_sleep(K_USEC(100));
            continue;
        }
        size_t frame_size = 0u;
        bool frame_ready = false;
        const asr_fc_hil_status_t status = asr_fc_hil_parser_push(
            &parser, byte, frame, sizeof(frame), &frame_size, &frame_ready);
        if (status != ASR_FC_HIL_OK) {
            asr_fc_flight_disarm(&flight_core);
            continue;
        }
        if (frame_ready) {
            process_frame(frame, frame_size);
        }
    }
}
