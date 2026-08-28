#ifndef ASR_FC_HIL_FLIGHT_ENDPOINT_H
#define ASR_FC_HIL_FLIGHT_ENDPOINT_H

#include <stddef.h>
#include <stdint.h>

#include "asr_fc/flight/flight_core.h"
#include "asr_fc/protocol/hil_link.h"

typedef struct {
    asr_fc_flight_config_t config;
    asr_fc_flight_core_t flight_core;
    uint32_t active_session;
    bool initialized;
} asr_fc_hil_flight_endpoint_t;

asr_fc_step_result_t asr_fc_hil_flight_endpoint_init(
    asr_fc_hil_flight_endpoint_t *endpoint,
    const asr_fc_flight_config_t *config);

asr_fc_hil_status_t asr_fc_hil_flight_endpoint_step(
    asr_fc_hil_flight_endpoint_t *endpoint,
    const uint8_t *frame,
    size_t frame_size,
    uint64_t device_timestamp_us,
    uint32_t *sequence,
    asr_fc_hil_flight_output_t *response);

#endif
