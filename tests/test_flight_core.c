#include "asr_fc/flight/flight_core.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static bool aiding_update(void *context,
                          const asr_fc_ceva_sample_t *ceva,
                          const asr_fc_aiding_sample_t *aiding,
                          double dt,
                          asr_fc_aided_state_t *state) {
    int *calls = context;
    (void)ceva;
    (void)dt;
    ++*calls;
    state->position[2] = aiding->tof_distance_m;
    return true;
}

static asr_fc_flight_config_t config(int *aiding_calls) {
    asr_fc_flight_config_t value = {
        .attitude_gains = {
            {.kp = 0.3, .ki = 0.01, .kd = 0.05,
             .integrator_limit = 0.2, .output_limit = 0.5},
            {.kp = 0.3, .ki = 0.01, .kd = 0.05,
             .integrator_limit = 0.2, .output_limit = 0.5},
            {.kp = 0.2, .ki = 0.0, .kd = 0.03,
             .integrator_limit = 0.0, .output_limit = 0.2},
        },
        .rate_weight = 1.0,
        .max_rotor_speed_rad_s = 10.0,
        .min_dt_s = 0.001,
        .max_dt_s = 0.050,
        .sensor_timeout_us = 50000u,
        .minimum_ceva_accuracy = 1u,
        .enabled_aiding_mask = ASR_FC_AID_BAROMETER | ASR_FC_AID_TOF,
        .aiding_update = aiding_update,
        .aiding_context = aiding_calls,
    };
    const double positions[4][3] = {
        {1.0, 1.0, 0.0}, {-1.0, 1.0, 0.0},
        {-1.0, -1.0, 0.0}, {1.0, -1.0, 0.0},
    };
    const double directions[4] = {1.0, -1.0, 1.0, -1.0};
    for (size_t index = 0; index < 4u; ++index) {
        memcpy(value.rotors[index].position, positions[index],
               sizeof(positions[index]));
        value.rotors[index].axis[2] = 1.0;
        value.rotors[index].direction = directions[index];
        value.rotors[index].thrust_coeff = 1.0;
        value.rotors[index].torque_coeff = 0.05;
    }
    return value;
}

static asr_fc_ceva_sample_t sample(uint32_t sequence, uint64_t timestamp_us) {
    asr_fc_ceva_sample_t value = {
        .sequence = sequence,
        .timestamp_us = timestamp_us,
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .accuracy = 3u,
        .attitude_valid = true,
    };
    return value;
}

static asr_fc_guidance_t guidance(bool arm) {
    asr_fc_guidance_t value = {
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .collective_thrust_n = 4.0,
        .arm_requested = arm,
    };
    return value;
}

static void test_ceva_direct_path_and_mixer(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    const asr_fc_flight_config_t cfg = config(&aiding_calls);
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);

    const asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    const asr_fc_guidance_t target = guidance(true);
    asr_fc_flight_output_t output;
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    assert(output.state == ASR_FC_FLIGHT_ARMED);
    assert(output.active_aiding_mask == 0u);
    assert(aiding_calls == 0);
    for (size_t index = 0; index < 4u; ++index) {
        assert(fabs(output.motor_speed_rad_s[index] - 1.0) < 1e-10);
        assert(output.motor_q15[index] > 3200u && output.motor_q15[index] < 3350u);
    }
}

static void test_attitude_correction_reaches_mixer(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    const asr_fc_flight_config_t cfg = config(&aiding_calls);
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    const double half_angle = 0.05;
    ceva.quaternion[0] = cos(half_angle);
    ceva.quaternion[1] = sin(half_angle);
    const asr_fc_guidance_t target = guidance(true);
    asr_fc_flight_output_t output;
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    assert(fabs(output.actuator.body_torque[0]) > 1e-4);
    assert(fabs(output.motor_speed_rad_s[0] -
                output.motor_speed_rad_s[2]) > 1e-4);
}

static void test_collective_follows_declared_rotor_axis(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    asr_fc_flight_config_t cfg = config(&aiding_calls);
    for (size_t index = 0u; index < 4u; ++index) {
        cfg.rotors[index].axis[2] = -1.0;
    }
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    const asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    const asr_fc_guidance_t target = guidance(true);
    asr_fc_flight_output_t output;
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    assert(output.actuator.collective_thrust == -4.0);
    for (size_t index = 0u; index < 4u; ++index) {
        assert(fabs(output.motor_speed_rad_s[index] - 1.0) < 1e-10);
    }
}

static void test_optional_aiding_is_explicit(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    const asr_fc_flight_config_t cfg = config(&aiding_calls);
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    const asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    const asr_fc_guidance_t target = guidance(true);
    const asr_fc_aiding_sample_t aiding = {
        .valid_mask = ASR_FC_AID_TOF,
        .tof_distance_m = 1.25,
    };
    asr_fc_flight_output_t output;
    assert(asr_fc_flight_step(&core, &ceva, &aiding, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    assert(aiding_calls == 1);
    assert(output.active_aiding_mask == ASR_FC_AID_TOF);
    assert(output.aided_state.position[2] == 1.25);
}

static void test_invalid_and_stale_samples_fail_closed(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    const asr_fc_flight_config_t cfg = config(&aiding_calls);
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    asr_fc_guidance_t target = guidance(true);
    asr_fc_flight_output_t output;

    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 160001u,
                              &output) == ASR_FC_STEP_SENSOR_STALE);
    assert(output.state == ASR_FC_FLIGHT_FAILSAFE);
    for (size_t index = 0; index < 4u; ++index) {
        assert(output.motor_q15[index] == 0u);
    }

    ceva = sample(2u, 170000u);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 170100u,
                              &output) == ASR_FC_STEP_FAILSAFE_LATCHED);
    target = guidance(false);
    ceva = sample(3u, 180000u);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 180100u,
                              &output) == ASR_FC_STEP_OK);
    assert(output.state == ASR_FC_FLIGHT_DISARMED);

    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    ceva.quaternion[0] = NAN;
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_SENSOR_INVALID);
    assert(output.state == ASR_FC_FLIGHT_FAILSAFE);
}

static void test_timing_and_disarm_contract(void) {
    int aiding_calls = 0;
    asr_fc_flight_core_t core;
    const asr_fc_flight_config_t cfg = config(&aiding_calls);
    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    asr_fc_ceva_sample_t ceva = sample(1u, 100000u);
    asr_fc_guidance_t target = guidance(false);
    asr_fc_flight_output_t output;
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    assert(output.state == ASR_FC_FLIGHT_DISARMED);

    target.arm_requested = true;
    ceva = sample(2u, 110000u);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 110100u,
                              &output) == ASR_FC_STEP_OK);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 110100u,
                              &output) == ASR_FC_STEP_SENSOR_INVALID);

    assert(asr_fc_flight_init(&core, &cfg) == ASR_FC_STEP_OK);
    ceva = sample(1u, 100000u);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 100100u,
                              &output) == ASR_FC_STEP_OK);
    ceva = sample(2u, 160001u);
    assert(asr_fc_flight_step(&core, &ceva, NULL, &target, 160100u,
                              &output) == ASR_FC_STEP_TIMING_INVALID);
    assert(output.state == ASR_FC_FLIGHT_FAILSAFE);
}

int main(void) {
    test_ceva_direct_path_and_mixer();
    test_attitude_correction_reaches_mixer();
    test_collective_follows_declared_rotor_axis();
    test_optional_aiding_is_explicit();
    test_invalid_and_stale_samples_fail_closed();
    test_timing_and_disarm_contract();
    puts("flight core tests passed");
    return 0;
}
