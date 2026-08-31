#include "asr_fc/flight/flight_core.h"
#include "drone/physics_model.h"

#include <math.h>
#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>
#include <zephyr/timing/timing.h>

#define BENCHMARK_STEPS 1000u
#define SAMPLE_PERIOD_US 10000u

static asr_fc_flight_core_t core;
static asr_fc_flight_config_t config;
static dm_vehicle_config_t vehicle_config;
static dm_vehicle_model_t plant;
static asr_fc_flight_output_t output;

static asr_fc_flight_config_t benchmark_config(void) {
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
    for (size_t index = 0; index < 4u; ++index) {
        memcpy(config.rotors[index].position, positions[index],
               sizeof(positions[index]));
        config.rotors[index].axis[2] = 1.0;
        config.rotors[index].direction = directions[index];
        config.rotors[index].thrust_coeff = 8.0e-6;
        config.rotors[index].torque_coeff = 1.2e-7;
    }
    return config;
}

static dm_vehicle_config_t plant_config(
    const asr_fc_flight_config_t *flight_config) {
    dm_vehicle_config_t config = {
        .rotor_count = 4u,
        .mass = 1.2,
        .gravity = 9.80665,
        .inertia = {{0.020, 0.0, 0.0},
                    {0.0, 0.020, 0.0},
                    {0.0, 0.0, 0.040}},
        .inertia_inv = {{50.0, 0.0, 0.0},
                        {0.0, 50.0, 0.0},
                        {0.0, 0.0, 25.0}},
    };
    for (size_t index = 0; index < 4u; ++index) {
        memcpy(config.rotors[index].position_body,
               flight_config->rotors[index].position,
               sizeof(config.rotors[index].position_body));
        memcpy(config.rotors[index].axis_body,
               flight_config->rotors[index].axis,
               sizeof(config.rotors[index].axis_body));
        config.rotors[index].direction = flight_config->rotors[index].direction;
        config.rotors[index].thrust_coeff =
            flight_config->rotors[index].thrust_coeff;
        config.rotors[index].torque_coeff =
            flight_config->rotors[index].torque_coeff;
    }
    return config;
}

static double attitude_error_degrees(const double quaternion[4]) {
    double scalar = fabs(quaternion[0]);
    if (scalar > 1.0) {
        scalar = 1.0;
    }
    return 2.0 * acos(scalar) * 180.0 / 3.14159265358979323846;
}

int main(void) {
    k_sleep(K_SECONDS(2));
    config = benchmark_config();
    if (asr_fc_flight_init(&core, &config) != ASR_FC_STEP_OK) {
        printk("ASR_FC_BENCH init_failed\n");
        return 1;
    }

    vehicle_config = plant_config(&config);
    plant = (dm_vehicle_model_t){
        .config = &vehicle_config,
        .state = {
            .quaternion = {0.9914448614, 0.1305261922, 0.0, 0.0},
        },
    };
    if (dm_vehicle_config_validate(&vehicle_config) != DM_OK) {
        printk("ASR_FC_BENCH plant_init_failed\n");
        return 2;
    }

    asr_fc_guidance_t guidance = {
        .quaternion = {1.0, 0.0, 0.0, 0.0},
        .collective_thrust_n = vehicle_config.mass * vehicle_config.gravity,
        .arm_requested = true,
    };
    timing_init();
    timing_start();
    uint64_t control_total_cycles = 0u;
    uint64_t control_worst_cycles = 0u;
    uint64_t loop_total_cycles = 0u;
    uint64_t loop_worst_cycles = 0u;
    const double initial_error_degrees =
        attitude_error_degrees(plant.state.quaternion);

    for (uint32_t index = 0u; index < BENCHMARK_STEPS; ++index) {
        asr_fc_ceva_sample_t sample = {
            .sequence = index + 1u,
            .timestamp_us = (uint64_t)(index + 1u) * SAMPLE_PERIOD_US,
            .received_at_us = (uint64_t)(index + 1u) * SAMPLE_PERIOD_US + 100u,
            .linear_acceleration = {0.0, 0.0, 0.0},
            .accuracy = 3u,
            .attitude_valid = true,
        };
        memcpy(sample.quaternion, plant.state.quaternion,
               sizeof(sample.quaternion));
        memcpy(sample.angular_rate, plant.state.angular_rate,
               sizeof(sample.angular_rate));

        timing_t loop_start = timing_counter_get();
        timing_t control_start = timing_counter_get();
        const asr_fc_step_result_t result = asr_fc_flight_step(
            &core, &sample, NULL, &guidance, sample.received_at_us,
            &output);
        timing_t control_end = timing_counter_get();
        if (result != ASR_FC_STEP_OK) {
            printk("ASR_FC_BENCH step_failed=%d index=%u\n", result, index);
            return 3;
        }
        double rotor_speed_rad_s[DM_MAX_ROTORS] = {0.0};
        memcpy(rotor_speed_rad_s, output.motor_speed_rad_s,
               sizeof(output.motor_speed_rad_s));
        if (dm_vehicle_step_rk4_checked(&plant, rotor_speed_rad_s,
                                        (double)SAMPLE_PERIOD_US / 1000000.0) !=
            DM_OK) {
            printk("ASR_FC_BENCH plant_step_failed index=%u\n", index);
            return 4;
        }
        timing_t loop_end = timing_counter_get();
        const uint64_t control_cycles =
            timing_cycles_get(&control_start, &control_end);
        const uint64_t loop_cycles = timing_cycles_get(&loop_start, &loop_end);
        control_total_cycles += control_cycles;
        loop_total_cycles += loop_cycles;
        if (control_cycles > control_worst_cycles) {
            control_worst_cycles = control_cycles;
        }
        if (loop_cycles > loop_worst_cycles) {
            loop_worst_cycles = loop_cycles;
        }
    }
    timing_stop();

    const double final_error_degrees =
        attitude_error_degrees(plant.state.quaternion);
    printk("ASR_FC_SIL steps=%u initial_error_mdeg=%d final_error_mdeg=%d "
           "final_rate_urad_s=%d control_avg_ns=%llu control_worst_ns=%llu "
           "loop_avg_ns=%llu loop_worst_ns=%llu motor_outputs_disabled=1\n",
           BENCHMARK_STEPS,
           (int)lround(initial_error_degrees * 1000.0),
           (int)lround(final_error_degrees * 1000.0),
           (int)lround(plant.state.angular_rate[0] * 1000000.0),
           timing_cycles_to_ns(control_total_cycles / BENCHMARK_STEPS),
           timing_cycles_to_ns(control_worst_cycles),
           timing_cycles_to_ns(loop_total_cycles / BENCHMARK_STEPS),
           timing_cycles_to_ns(loop_worst_cycles));
    for (;;) {
        k_sleep(K_SECONDS(1));
    }
}
