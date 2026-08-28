#ifndef ASR_FC_SENSORS_CEVA_FSM300_ADAPTER_H
#define ASR_FC_SENSORS_CEVA_FSM300_ADAPTER_H

#include <stdbool.h>
#include <stdint.h>

#include "asr_fc/flight/flight_core.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    ASR_FC_CEVA_ADAPTER_OK = 0,
    ASR_FC_CEVA_ADAPTER_INVALID_ARGUMENT,
    ASR_FC_CEVA_ADAPTER_INVALID_VALUE,
    ASR_FC_CEVA_ADAPTER_REPEATED_REPORT,
    ASR_FC_CEVA_ADAPTER_INCOMPLETE,
    ASR_FC_CEVA_ADAPTER_STALE,
    ASR_FC_CEVA_ADAPTER_COMPONENT_SKEW
} asr_fc_ceva_adapter_result_t;

typedef struct {
    uint32_t max_component_age_us;
    uint32_t max_component_skew_us;
    double minimum_quaternion_norm;
    double maximum_quaternion_norm;
} asr_fc_ceva_adapter_config_t;

typedef struct {
    asr_fc_ceva_adapter_config_t config;
    double quaternion[4];
    double angular_rate[3];
    double last_emitted_quaternion[4];
    uint64_t quaternion_timestamp_us;
    uint64_t gyro_timestamp_us;
    uint64_t emitted_quaternion_timestamp_us;
    uint64_t emitted_gyro_timestamp_us;
    uint32_t sequence;
    uint8_t accuracy;
    bool has_quaternion;
    bool has_gyro;
    bool has_emitted_quaternion;
} asr_fc_ceva_adapter_t;

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_init(
    asr_fc_ceva_adapter_t *adapter,
    const asr_fc_ceva_adapter_config_t *config);

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_update_rotation(
    asr_fc_ceva_adapter_t *adapter,
    const double quaternion_scalar_first[4],
    uint8_t accuracy,
    uint64_t timestamp_us);

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_update_gyro(
    asr_fc_ceva_adapter_t *adapter,
    const double angular_rate_rad_s[3],
    uint64_t timestamp_us);

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_take_sample(
    asr_fc_ceva_adapter_t *adapter,
    uint64_t now_us,
    asr_fc_ceva_sample_t *sample);

#ifdef __cplusplus
}
#endif

#endif
