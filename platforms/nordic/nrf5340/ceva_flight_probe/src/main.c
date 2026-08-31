#include "ceva_spi_hal.h"

#include "asr_fc/flight/flight_core.h"
#include "asr_fc/sensors/ceva_fsm300_adapter.h"
#include "sh2.h"
#include "sh2_SensorValue.h"
#include "sh2_err.h"

#include <string.h>

#include <zephyr/kernel.h>
#include <zephyr/sys/printk.h>

#define SENSOR_PERIOD_US 40000u
#define DIAGNOSTIC_PERIOD 25u

enum { ASR_FC_PHYSICAL_OUTPUTS_ENABLED = 0 };
BUILD_ASSERT(ASR_FC_PHYSICAL_OUTPUTS_ENABLED == 0,
             "The CEVA flight probe must not drive physical outputs");

static asr_fc_ceva_adapter_t adapter;
static asr_fc_flight_core_t flight_core;
static asr_fc_guidance_t guidance;
static bool guidance_initialized;
static uint32_t decoded_reports;
static uint32_t complete_samples;
static uint32_t rejected_reports;

static uint64_t host_monotonic_us(void) {
    return k_ticks_to_us_floor64(k_uptime_ticks());
}

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
        .min_dt_s = 0.020,
        .max_dt_s = 0.060,
        .sensor_timeout_us = 100000u,
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

static void async_event(void *cookie, sh2_AsyncEvent_t *event) {
    ARG_UNUSED(cookie);
    printk("ASR_FC_CEVA async_event=%u\n", event->eventId);
}

static void process_complete_sample(uint64_t now_us) {
    asr_fc_ceva_sample_t sample;
    const asr_fc_ceva_adapter_result_t adapter_result =
        asr_fc_ceva_adapter_take_sample(&adapter, now_us, &sample);
    if (adapter_result != ASR_FC_CEVA_ADAPTER_OK) {
        return;
    }
    if (!guidance_initialized) {
        memcpy(guidance.quaternion, sample.quaternion,
               sizeof(guidance.quaternion));
        guidance.collective_thrust_n = 11.76798;
        guidance.arm_requested = true;
        guidance_initialized = true;
    }

    asr_fc_flight_output_t output;
    const asr_fc_step_result_t step = asr_fc_flight_step(
        &flight_core, &sample, NULL, &guidance, now_us, &output);
    complete_samples += 1u;
    if (complete_samples % DIAGNOSTIC_PERIOD == 0u ||
        step != ASR_FC_STEP_OK) {
        printk("ASR_FC_CEVA sample=%u step=%d status=%u "
               "q=%d,%d,%d,%d gyro_mrad=%d,%d,%d "
               "virtual_q15=%u,%u,%u,%u physical_outputs=0\n",
               sample.sequence, step, sample.accuracy,
               (int)(sample.quaternion[0] * 1000000.0),
               (int)(sample.quaternion[1] * 1000000.0),
               (int)(sample.quaternion[2] * 1000000.0),
               (int)(sample.quaternion[3] * 1000000.0),
               (int)(sample.angular_rate[0] * 1000.0),
               (int)(sample.angular_rate[1] * 1000.0),
               (int)(sample.angular_rate[2] * 1000.0),
               output.motor_q15[0], output.motor_q15[1],
               output.motor_q15[2], output.motor_q15[3]);
    }
}

static void sensor_event(void *cookie, sh2_SensorEvent_t *event) {
    ARG_UNUSED(cookie);
    sh2_SensorValue_t value;
    if (sh2_decodeSensorEvent(&value, event) != SH2_OK) {
        rejected_reports += 1u;
        return;
    }
    decoded_reports += 1u;
    const uint64_t received_at_us = host_monotonic_us();

    asr_fc_ceva_adapter_result_t result =
        ASR_FC_CEVA_ADAPTER_INVALID_VALUE;
    if (value.sensorId == SH2_GAME_ROTATION_VECTOR) {
        const double quaternion[4] = {
            value.un.gameRotationVector.real,
            value.un.gameRotationVector.i,
            value.un.gameRotationVector.j,
            value.un.gameRotationVector.k,
        };
        result = asr_fc_ceva_adapter_update_rotation(
            &adapter, quaternion, value.status, value.timestamp,
            received_at_us);
    } else if (value.sensorId == SH2_GYROSCOPE_CALIBRATED) {
        const double angular_rate[3] = {
            value.un.gyroscope.x,
            value.un.gyroscope.y,
            value.un.gyroscope.z,
        };
        result = asr_fc_ceva_adapter_update_gyro(
            &adapter, angular_rate, value.timestamp, received_at_us);
    } else {
        return;
    }
    if (result != ASR_FC_CEVA_ADAPTER_OK) {
        rejected_reports += 1u;
        return;
    }
    process_complete_sample(host_monotonic_us());
}

static int enable_report(sh2_SensorId_t sensor_id) {
    const sh2_SensorConfig_t config = {
        .reportInterval_us = SENSOR_PERIOD_US,
        .batchInterval_us = 0u,
        .sensorSpecific = 0u,
    };
    return sh2_setSensorConfig(sensor_id, &config);
}

int main(void) {
    k_sleep(K_SECONDS(2));
    printk("ASR_FC_CEVA_PROBE boot physical_outputs=0 pwm=absent esc=absent\n");

    const asr_fc_ceva_adapter_config_t adapter_config = {
        .max_component_age_us = 100000u,
        .max_component_skew_us = 25000u,
        .minimum_quaternion_norm = 0.90,
        .maximum_quaternion_norm = 1.10,
    };
    asr_fc_flight_config_t config = flight_config();
    if (asr_fc_ceva_adapter_init(&adapter, &adapter_config) !=
            ASR_FC_CEVA_ADAPTER_OK ||
        asr_fc_flight_init(&flight_core, &config) != ASR_FC_STEP_OK) {
        printk("ASR_FC_CEVA_PROBE init_failed\n");
        return 1;
    }
    int result = sh2_open(asr_fc_ceva_spi_hal(), async_event, NULL);
    if (result != SH2_OK) {
        printk("ASR_FC_CEVA_PROBE sh2_open_failed=%d\n", result);
        return 2;
    }
    (void)sh2_setSensorCallback(sensor_event, NULL);
    result = enable_report(SH2_GAME_ROTATION_VECTOR);
    if (result == SH2_OK) {
        result = enable_report(SH2_GYROSCOPE_CALIBRATED);
    }
    if (result != SH2_OK) {
        printk("ASR_FC_CEVA_PROBE report_enable_failed=%d\n", result);
        sh2_close();
        return 3;
    }
    printk("ASR_FC_CEVA_PROBE reports_enabled period_us=%u\n",
           SENSOR_PERIOD_US);

    uint32_t last_health_ms = 0u;
    for (;;) {
        sh2_service();
        const uint32_t now_ms = k_uptime_get_32();
        if (now_ms - last_health_ms >= 5000u) {
            last_health_ms = now_ms;
            printk("ASR_FC_CEVA_HEALTH decoded=%u complete=%u rejected=%u "
                   "physical_outputs=0\n",
                   decoded_reports, complete_samples, rejected_reports);
        }
        k_sleep(K_MSEC(1));
    }
}
