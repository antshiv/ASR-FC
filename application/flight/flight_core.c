#include "asr_fc/flight/flight_core.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static void safe_output(asr_fc_flight_output_t *output,
                        asr_fc_flight_state_t state) {
    if (output == NULL) {
        return;
    }
    memset(output, 0, sizeof(*output));
    output->state = state;
}

static bool finite_vector(const double *values, size_t count) {
    for (size_t index = 0; index < count; ++index) {
        if (!isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

static bool valid_quaternion(const double quaternion[4]) {
    if (!finite_vector(quaternion, 4u)) {
        return false;
    }
    double norm_squared = 0.0;
    for (size_t index = 0; index < 4u; ++index) {
        norm_squared += quaternion[index] * quaternion[index];
    }
    return isfinite(norm_squared) && norm_squared > 0.81 && norm_squared < 1.21;
}

static bool collective_axis_sign(const asr_fc_flight_config_t *config,
                                 double *sign) {
    const double first = config->rotors[0].axis[2];
    if (!isfinite(first) || fabs(first) < 1e-9) {
        return false;
    }
    *sign = first < 0.0 ? -1.0 : 1.0;
    for (size_t index = 1u; index < 4u; ++index) {
        const double axis_z = config->rotors[index].axis[2];
        if (!isfinite(axis_z) || axis_z * *sign <= 0.0) {
            return false;
        }
    }
    return true;
}

static bool valid_config(const asr_fc_flight_config_t *config) {
    double unused_collective_sign = 0.0;
    return config != NULL &&
           collective_axis_sign(config, &unused_collective_sign) &&
           isfinite(config->rate_weight) &&
           config->rate_weight >= 0.0 &&
           isfinite(config->max_rotor_speed_rad_s) &&
           config->max_rotor_speed_rad_s > 0.0 &&
           isfinite(config->min_dt_s) && config->min_dt_s > 0.0 &&
           isfinite(config->max_dt_s) &&
           config->max_dt_s >= config->min_dt_s &&
           config->sensor_timeout_us > 0u &&
           (config->enabled_aiding_mask == 0u || config->aiding_update != NULL);
}

static void enter_failsafe(asr_fc_flight_core_t *core,
                           asr_fc_flight_output_t *output) {
    core->state = ASR_FC_FLIGHT_FAILSAFE;
    cs_attitude_pid_reset(&core->attitude_controller);
    safe_output(output, core->state);
}

static void enter_disarmed(asr_fc_flight_core_t *core,
                           asr_fc_flight_output_t *output) {
    core->state = ASR_FC_FLIGHT_DISARMED;
    cs_attitude_pid_reset(&core->attitude_controller);
    safe_output(output, core->state);
}

asr_fc_step_result_t asr_fc_flight_init(
    asr_fc_flight_core_t *core,
    const asr_fc_flight_config_t *config) {
    if (core == NULL || config == NULL) {
        return ASR_FC_STEP_INVALID_ARGUMENT;
    }
    memset(core, 0, sizeof(*core));
    if (!valid_config(config)) {
        return ASR_FC_STEP_INVALID_CONFIG;
    }
    core->config = *config;
    if (!cs_attitude_pid_init_checked(&core->attitude_controller,
                                      config->attitude_gains,
                                      config->rate_weight)) {
        memset(core, 0, sizeof(*core));
        return ASR_FC_STEP_INVALID_CONFIG;
    }
    if (cs_mixer_init(&core->mixer, config->rotors, 4u) != 0) {
        memset(core, 0, sizeof(*core));
        return ASR_FC_STEP_INVALID_CONFIG;
    }
    core->state = ASR_FC_FLIGHT_DISARMED;
    return ASR_FC_STEP_OK;
}

void asr_fc_flight_disarm(asr_fc_flight_core_t *core) {
    if (core == NULL) {
        return;
    }
    core->state = ASR_FC_FLIGHT_DISARMED;
    core->has_sample = false;
    cs_attitude_pid_reset(&core->attitude_controller);
}

asr_fc_step_result_t asr_fc_flight_step(
    asr_fc_flight_core_t *core,
    const asr_fc_ceva_sample_t *ceva,
    const asr_fc_aiding_sample_t *aiding,
    const asr_fc_guidance_t *guidance,
    uint64_t now_us,
    asr_fc_flight_output_t *output) {
    if (core == NULL || ceva == NULL || guidance == NULL || output == NULL) {
        return ASR_FC_STEP_INVALID_ARGUMENT;
    }
    safe_output(output, core->state);

    if (!ceva->attitude_valid || ceva->accuracy < core->config.minimum_ceva_accuracy ||
        !valid_quaternion(ceva->quaternion) ||
        !finite_vector(ceva->angular_rate, 3u) ||
        !finite_vector(ceva->linear_acceleration, 3u)) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_SENSOR_INVALID;
    }
    if (ceva->received_at_us == 0u || now_us < ceva->received_at_us ||
        now_us - ceva->received_at_us > core->config.sensor_timeout_us) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_SENSOR_STALE;
    }
    if (core->has_sample && (ceva->sequence <= core->last_sequence ||
        ceva->timestamp_us <= core->last_sample_timestamp_us)) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_SENSOR_INVALID;
    }

    double dt = core->config.min_dt_s;
    if (core->has_sample) {
        dt = (double)(ceva->timestamp_us - core->last_sample_timestamp_us) / 1000000.0;
        if (!isfinite(dt) || dt < core->config.min_dt_s ||
            dt > core->config.max_dt_s) {
            enter_failsafe(core, output);
            return ASR_FC_STEP_TIMING_INVALID;
        }
    }
    core->last_sequence = ceva->sequence;
    core->last_sample_timestamp_us = ceva->timestamp_us;
    core->has_sample = true;

    if (core->state == ASR_FC_FLIGHT_FAILSAFE) {
        if (guidance->arm_requested) {
            safe_output(output, core->state);
            return ASR_FC_STEP_FAILSAFE_LATCHED;
        }
        enter_disarmed(core, output);
        return ASR_FC_STEP_OK;
    }
    if (!guidance->arm_requested) {
        enter_disarmed(core, output);
        return ASR_FC_STEP_OK;
    }
    if (!valid_quaternion(guidance->quaternion) ||
        !finite_vector(guidance->angular_rate, 3u) ||
        !isfinite(guidance->collective_thrust_n) ||
        guidance->collective_thrust_n < 0.0) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_CONTROL_FAILED;
    }

    if (core->state != ASR_FC_FLIGHT_ARMED) {
        core->state = ASR_FC_FLIGHT_ARMED;
        cs_attitude_pid_reset(&core->attitude_controller);
    }

    const uint32_t active_aiding = aiding == NULL ? 0u :
        (aiding->valid_mask & core->config.enabled_aiding_mask);
    if (active_aiding != 0u) {
        if (!core->config.aiding_update(core->config.aiding_context, ceva, aiding,
                                        dt, &output->aided_state)) {
            enter_failsafe(core, output);
            return ASR_FC_STEP_AIDING_FAILED;
        }
        output->active_aiding_mask = active_aiding;
    }

    cs_state_t state = {0};
    memcpy(state.quaternion, ceva->quaternion, sizeof(state.quaternion));
    memcpy(state.angular_rate, ceva->angular_rate, sizeof(state.angular_rate));
    cs_attitude_setpoint_t setpoint = {0};
    memcpy(setpoint.quaternion, guidance->quaternion, sizeof(setpoint.quaternion));
    memcpy(setpoint.angular_rate, guidance->angular_rate,
           sizeof(setpoint.angular_rate));

    if (!cs_attitude_pid_update_checked(&core->attitude_controller, &setpoint,
                                        &state, dt, &output->actuator)) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_CONTROL_FAILED;
    }
    double collective_sign = 0.0;
    if (!collective_axis_sign(&core->config, &collective_sign)) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_CONTROL_FAILED;
    }
    output->actuator.collective_thrust =
        guidance->collective_thrust_n * collective_sign;
    if (cs_mixer_mix(&core->mixer, &output->actuator,
                     output->motor_speed_rad_s) != 0) {
        enter_failsafe(core, output);
        return ASR_FC_STEP_MIXER_FAILED;
    }
    for (size_t index = 0; index < 4u; ++index) {
        double normalized = output->motor_speed_rad_s[index] /
                            core->config.max_rotor_speed_rad_s;
        if (!isfinite(normalized)) {
            enter_failsafe(core, output);
            return ASR_FC_STEP_MIXER_FAILED;
        }
        if (normalized > 1.0) {
            normalized = 1.0;
        }
        output->motor_q15[index] = (uint16_t)lround(normalized * 32767.0);
    }
    output->state = core->state;
    return ASR_FC_STEP_OK;
}
