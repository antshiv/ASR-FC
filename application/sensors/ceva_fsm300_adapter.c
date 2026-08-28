#include "asr_fc/sensors/ceva_fsm300_adapter.h"

#include <math.h>
#include <stddef.h>
#include <string.h>

static bool finite_vector(const double *values, size_t count) {
    for (size_t index = 0; index < count; ++index) {
        if (!isfinite(values[index])) {
            return false;
        }
    }
    return true;
}

static bool valid_config(const asr_fc_ceva_adapter_config_t *config) {
    return config != NULL && config->max_component_age_us > 0u &&
           config->max_component_skew_us > 0u &&
           isfinite(config->minimum_quaternion_norm) &&
           config->minimum_quaternion_norm > 0.0 &&
           isfinite(config->maximum_quaternion_norm) &&
           config->maximum_quaternion_norm >= config->minimum_quaternion_norm;
}

static uint64_t absolute_difference(uint64_t left, uint64_t right) {
    return left >= right ? left - right : right - left;
}

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_init(
    asr_fc_ceva_adapter_t *adapter,
    const asr_fc_ceva_adapter_config_t *config) {
    if (adapter == NULL || config == NULL) {
        return ASR_FC_CEVA_ADAPTER_INVALID_ARGUMENT;
    }
    memset(adapter, 0, sizeof(*adapter));
    if (!valid_config(config)) {
        return ASR_FC_CEVA_ADAPTER_INVALID_VALUE;
    }
    adapter->config = *config;
    return ASR_FC_CEVA_ADAPTER_OK;
}

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_update_rotation(
    asr_fc_ceva_adapter_t *adapter,
    const double quaternion_scalar_first[4],
    uint8_t accuracy,
    uint64_t timestamp_us) {
    if (adapter == NULL || quaternion_scalar_first == NULL) {
        return ASR_FC_CEVA_ADAPTER_INVALID_ARGUMENT;
    }
    if (!finite_vector(quaternion_scalar_first, 4u) || timestamp_us == 0u) {
        return ASR_FC_CEVA_ADAPTER_INVALID_VALUE;
    }
    if (adapter->has_quaternion &&
        timestamp_us <= adapter->quaternion_timestamp_us) {
        return ASR_FC_CEVA_ADAPTER_REPEATED_REPORT;
    }

    double norm_squared = 0.0;
    for (size_t index = 0; index < 4u; ++index) {
        norm_squared += quaternion_scalar_first[index] *
                        quaternion_scalar_first[index];
    }
    const double norm = sqrt(norm_squared);
    if (!isfinite(norm) || norm < adapter->config.minimum_quaternion_norm ||
        norm > adapter->config.maximum_quaternion_norm) {
        return ASR_FC_CEVA_ADAPTER_INVALID_VALUE;
    }
    for (size_t index = 0; index < 4u; ++index) {
        adapter->quaternion[index] = quaternion_scalar_first[index] / norm;
    }
    adapter->accuracy = accuracy;
    adapter->quaternion_timestamp_us = timestamp_us;
    adapter->has_quaternion = true;
    return ASR_FC_CEVA_ADAPTER_OK;
}

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_update_gyro(
    asr_fc_ceva_adapter_t *adapter,
    const double angular_rate_rad_s[3],
    uint64_t timestamp_us) {
    if (adapter == NULL || angular_rate_rad_s == NULL) {
        return ASR_FC_CEVA_ADAPTER_INVALID_ARGUMENT;
    }
    if (!finite_vector(angular_rate_rad_s, 3u) || timestamp_us == 0u) {
        return ASR_FC_CEVA_ADAPTER_INVALID_VALUE;
    }
    if (adapter->has_gyro && timestamp_us <= adapter->gyro_timestamp_us) {
        return ASR_FC_CEVA_ADAPTER_REPEATED_REPORT;
    }
    memcpy(adapter->angular_rate, angular_rate_rad_s,
           sizeof(adapter->angular_rate));
    adapter->gyro_timestamp_us = timestamp_us;
    adapter->has_gyro = true;
    return ASR_FC_CEVA_ADAPTER_OK;
}

asr_fc_ceva_adapter_result_t asr_fc_ceva_adapter_take_sample(
    asr_fc_ceva_adapter_t *adapter,
    uint64_t now_us,
    asr_fc_ceva_sample_t *sample) {
    if (adapter == NULL || sample == NULL) {
        return ASR_FC_CEVA_ADAPTER_INVALID_ARGUMENT;
    }
    memset(sample, 0, sizeof(*sample));
    if (!adapter->has_quaternion || !adapter->has_gyro) {
        return ASR_FC_CEVA_ADAPTER_INCOMPLETE;
    }
    if (adapter->quaternion_timestamp_us <=
            adapter->emitted_quaternion_timestamp_us ||
        adapter->gyro_timestamp_us <= adapter->emitted_gyro_timestamp_us) {
        return ASR_FC_CEVA_ADAPTER_INCOMPLETE;
    }
    if (now_us < adapter->quaternion_timestamp_us ||
        now_us < adapter->gyro_timestamp_us ||
        now_us - adapter->quaternion_timestamp_us >
            adapter->config.max_component_age_us ||
        now_us - adapter->gyro_timestamp_us >
            adapter->config.max_component_age_us) {
        return ASR_FC_CEVA_ADAPTER_STALE;
    }
    if (absolute_difference(adapter->quaternion_timestamp_us,
                            adapter->gyro_timestamp_us) >
        adapter->config.max_component_skew_us) {
        return ASR_FC_CEVA_ADAPTER_COMPONENT_SKEW;
    }

    double output_quaternion[4];
    memcpy(output_quaternion, adapter->quaternion,
           sizeof(output_quaternion));
    if (adapter->has_emitted_quaternion) {
        double dot = 0.0;
        for (size_t index = 0; index < 4u; ++index) {
            dot += output_quaternion[index] *
                   adapter->last_emitted_quaternion[index];
        }
        if (dot < 0.0) {
            for (size_t index = 0; index < 4u; ++index) {
                output_quaternion[index] = -output_quaternion[index];
            }
        }
    }

    adapter->sequence += 1u;
    sample->sequence = adapter->sequence;
    sample->timestamp_us = adapter->quaternion_timestamp_us >=
                                   adapter->gyro_timestamp_us
                               ? adapter->quaternion_timestamp_us
                               : adapter->gyro_timestamp_us;
    memcpy(sample->quaternion, output_quaternion, sizeof(sample->quaternion));
    memcpy(sample->angular_rate, adapter->angular_rate,
           sizeof(sample->angular_rate));
    sample->accuracy = adapter->accuracy;
    sample->attitude_valid = true;

    memcpy(adapter->last_emitted_quaternion, output_quaternion,
           sizeof(adapter->last_emitted_quaternion));
    adapter->has_emitted_quaternion = true;
    adapter->emitted_quaternion_timestamp_us =
        adapter->quaternion_timestamp_us;
    adapter->emitted_gyro_timestamp_us = adapter->gyro_timestamp_us;
    return ASR_FC_CEVA_ADAPTER_OK;
}
