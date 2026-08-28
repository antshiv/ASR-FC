#include "asr_fc/hil/flight_endpoint.h"

#include <string.h>

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

asr_fc_step_result_t asr_fc_hil_flight_endpoint_init(
    asr_fc_hil_flight_endpoint_t *endpoint,
    const asr_fc_flight_config_t *config) {
    if (endpoint == NULL || config == NULL) {
        return ASR_FC_STEP_INVALID_ARGUMENT;
    }
    memset(endpoint, 0, sizeof(*endpoint));
    endpoint->config = *config;
    const asr_fc_step_result_t result = asr_fc_flight_init(
        &endpoint->flight_core, &endpoint->config);
    endpoint->initialized = result == ASR_FC_STEP_OK;
    return result;
}

asr_fc_hil_status_t asr_fc_hil_flight_endpoint_step(
    asr_fc_hil_flight_endpoint_t *endpoint,
    const uint8_t *frame,
    size_t frame_size,
    uint64_t device_timestamp_us,
    uint32_t *sequence,
    asr_fc_hil_flight_output_t *response) {
    if (endpoint == NULL || sequence == NULL || response == NULL) {
        return ASR_FC_HIL_NULL_POINTER;
    }
    memset(response, 0, sizeof(*response));
    if (!endpoint->initialized) {
        return ASR_FC_HIL_INVALID_FRAME;
    }

    asr_fc_hil_sensor_guidance_t request;
    const asr_fc_hil_status_t decode = asr_fc_hil_decode_sensor_guidance(
        frame, frame_size, sequence, &request);
    if (decode != ASR_FC_HIL_OK) {
        asr_fc_flight_disarm(&endpoint->flight_core);
        return decode;
    }
    if (request.session_id != endpoint->active_session) {
        const asr_fc_step_result_t reset = asr_fc_flight_init(
            &endpoint->flight_core, &endpoint->config);
        if (reset != ASR_FC_STEP_OK) {
            endpoint->initialized = false;
            return ASR_FC_HIL_INVALID_FRAME;
        }
        endpoint->active_session = request.session_id;
    }

    asr_fc_ceva_sample_t sample = {
        .sequence = *sequence,
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
    const asr_fc_step_result_t step = asr_fc_flight_step(
        &endpoint->flight_core, &sample, NULL, &guidance,
        request.sensor_timestamp_us, &output);

    *response = (asr_fc_hil_flight_output_t){
        .session_id = request.session_id,
        .acknowledged_sequence = *sequence,
        .device_timestamp_us = device_timestamp_us,
        .fault_flags = step == ASR_FC_STEP_OK ? 0u :
            (1u << (uint32_t)step),
        .active_aiding_mask = output.active_aiding_mask,
        .step_result = (uint8_t)step,
        .flight_state = (uint8_t)output.state,
        .collective_thrust_n = (float)output.actuator.collective_thrust,
    };
    memcpy(response->motor_q15, output.motor_q15,
           sizeof(response->motor_q15));
    copy_double_to_float(response->motor_speed_rad_s,
                         output.motor_speed_rad_s, 4u);
    copy_double_to_float(response->body_torque_nm,
                         output.actuator.body_torque, 3u);
    memcpy(response->observed_quaternion, request.quaternion,
           sizeof(response->observed_quaternion));
    memcpy(response->observed_angular_rate, request.angular_rate,
           sizeof(response->observed_angular_rate));
    memcpy(response->observed_linear_acceleration,
           request.linear_acceleration,
           sizeof(response->observed_linear_acceleration));
    return ASR_FC_HIL_OK;
}
