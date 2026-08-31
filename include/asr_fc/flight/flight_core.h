#ifndef ASR_FC_FLIGHT_CORE_H
#define ASR_FC_FLIGHT_CORE_H

#include <stdbool.h>
#include <stdint.h>

#include "mixer.h"
#include "pid.h"

#ifdef __cplusplus
extern "C" {
#endif

#define ASR_FC_AID_BAROMETER (1u << 0)
#define ASR_FC_AID_TOF       (1u << 1)
#define ASR_FC_AID_GNSS      (1u << 2)

typedef enum {
    ASR_FC_FLIGHT_DISARMED = 0,
    ASR_FC_FLIGHT_ARMED = 1,
    ASR_FC_FLIGHT_FAILSAFE = 2
} asr_fc_flight_state_t;

typedef enum {
    ASR_FC_STEP_OK = 0,
    ASR_FC_STEP_INVALID_ARGUMENT = 1,
    ASR_FC_STEP_INVALID_CONFIG = 2,
    ASR_FC_STEP_SENSOR_INVALID = 3,
    ASR_FC_STEP_SENSOR_STALE = 4,
    ASR_FC_STEP_TIMING_INVALID = 5,
    ASR_FC_STEP_CONTROL_FAILED = 6,
    ASR_FC_STEP_MIXER_FAILED = 7,
    ASR_FC_STEP_AIDING_FAILED = 8,
    ASR_FC_STEP_FAILSAFE_LATCHED = 9
} asr_fc_step_result_t;

typedef struct {
    uint32_t sequence;
    uint64_t timestamp_us; /* Sensor clock: ordering and control-loop dt. */
    uint64_t received_at_us; /* Host monotonic clock: freshness only. */
    double quaternion[4]; /* CEVA scalar-first body-to-inertial quaternion. */
    double angular_rate[3];
    double linear_acceleration[3];
    uint8_t accuracy;
    bool attitude_valid;
} asr_fc_ceva_sample_t;

typedef struct {
    uint32_t valid_mask;
    double barometer_pressure_pa;
    double tof_distance_m;
    double gnss_position_m[3];
} asr_fc_aiding_sample_t;

typedef struct {
    double quaternion[4];
    double angular_rate[3];
    double collective_thrust_n; /* Non-negative magnitude; rotor axes own sign. */
    bool arm_requested;
} asr_fc_guidance_t;

typedef struct {
    double position[3];
    double velocity[3];
} asr_fc_aided_state_t;

typedef bool (*asr_fc_aiding_update_fn)(
    void *context,
    const asr_fc_ceva_sample_t *ceva,
    const asr_fc_aiding_sample_t *aiding,
    double dt,
    asr_fc_aided_state_t *state);

typedef struct {
    cs_pid_gains_t attitude_gains[3];
    double rate_weight;
    cs_rotor_config_t rotors[4];
    double max_rotor_speed_rad_s;
    double min_dt_s;
    double max_dt_s;
    uint32_t sensor_timeout_us;
    uint8_t minimum_ceva_accuracy;
    uint32_t enabled_aiding_mask;
    asr_fc_aiding_update_fn aiding_update;
    void *aiding_context;
} asr_fc_flight_config_t;

typedef struct {
    asr_fc_flight_config_t config;
    cs_attitude_pid_t attitude_controller;
    cs_mixer_t mixer;
    asr_fc_flight_state_t state;
    uint32_t last_sequence;
    uint64_t last_sample_timestamp_us;
    bool has_sample;
} asr_fc_flight_core_t;

typedef struct {
    asr_fc_flight_state_t state;
    uint16_t motor_q15[4];
    double motor_speed_rad_s[4];
    cs_actuator_command_t actuator;
    asr_fc_aided_state_t aided_state;
    uint32_t active_aiding_mask;
} asr_fc_flight_output_t;

asr_fc_step_result_t asr_fc_flight_init(
    asr_fc_flight_core_t *core,
    const asr_fc_flight_config_t *config);

asr_fc_step_result_t asr_fc_flight_step(
    asr_fc_flight_core_t *core,
    const asr_fc_ceva_sample_t *ceva,
    const asr_fc_aiding_sample_t *aiding,
    const asr_fc_guidance_t *guidance,
    uint64_t now_us,
    asr_fc_flight_output_t *output);

void asr_fc_flight_disarm(asr_fc_flight_core_t *core);

#ifdef __cplusplus
}
#endif

#endif
