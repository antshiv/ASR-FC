#ifndef ASR_FC_PLATFORM_GD3000_PROTOCOL_H
#define ASR_FC_PLATFORM_GD3000_PROTOCOL_H

#include <stdbool.h>
#include <stdint.h>

typedef enum {
    ASR_FC_GD3000_OK = 0,
    ASR_FC_GD3000_NULL_POINTER,
    ASR_FC_GD3000_INVALID_STATUS_REGISTER
} asr_fc_gd3000_status_t;

typedef struct {
    bool overtemperature;
    bool desaturation;
    bool low_vls;
    bool overcurrent;
    bool phase_error;
    bool framing_error;
    bool write_error;
    bool reset_event;
} asr_fc_gd3000_status0_t;

typedef struct {
    uint8_t request;
    uint8_t flush;
} asr_fc_gd3000_status_read_t;

/* NULL0 through NULL3 are read-only and do not alter GD3000 operation. */
asr_fc_gd3000_status_t asr_fc_gd3000_null_command(
    uint8_t status_register,
    uint8_t *command);

/*
 * GD3000 responses are pipelined. The response to request is returned while
 * flush is transmitted in a second, separately chip-selected transaction.
 */
asr_fc_gd3000_status_t asr_fc_gd3000_status_read_sequence(
    uint8_t status_register,
    asr_fc_gd3000_status_read_t *sequence);

asr_fc_gd3000_status_t asr_fc_gd3000_decode_status0(
    uint8_t value,
    asr_fc_gd3000_status0_t *status);

bool asr_fc_gd3000_status0_has_fault(
    const asr_fc_gd3000_status0_t *status);

#endif
