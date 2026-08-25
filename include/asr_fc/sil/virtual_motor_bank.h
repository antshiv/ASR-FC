#ifndef ASR_FC_SIL_VIRTUAL_MOTOR_BANK_H
#define ASR_FC_SIL_VIRTUAL_MOTOR_BANK_H

#include <stdbool.h>
#include <stdint.h>

#include "asr_fc/protocol/motor_link.h"

#define ASR_FC_MOTOR_COUNT 4u
#define ASR_FC_VIRTUAL_MOTOR_FAULT_COMMAND_TIMEOUT (1u << 0)
#define ASR_FC_VIRTUAL_MOTOR_FAULT_SEQUENCE (1u << 1)
#define ASR_FC_VIRTUAL_MOTOR_FAULT_SESSION (1u << 2)

typedef struct {
    bool initialized;
    bool armed;
    uint16_t throttle_q15;
    uint16_t command_timeout_ms;
    uint32_t elapsed_ms;
    uint32_t session_id;
    uint32_t last_sequence;
    uint32_t fault_flags;
} asr_fc_virtual_motor_t;

typedef struct {
    asr_fc_virtual_motor_t motor[ASR_FC_MOTOR_COUNT];
} asr_fc_virtual_motor_bank_t;

void asr_fc_virtual_motor_bank_init(asr_fc_virtual_motor_bank_t *bank);

asr_fc_motor_link_status_t asr_fc_virtual_motor_bank_apply(
    asr_fc_virtual_motor_bank_t *bank,
    uint32_t sequence,
    const asr_fc_motor_command_t *command);

void asr_fc_virtual_motor_bank_step(
    asr_fc_virtual_motor_bank_t *bank,
    uint32_t elapsed_ms);

asr_fc_motor_link_status_t asr_fc_virtual_motor_bank_telemetry(
    const asr_fc_virtual_motor_bank_t *bank,
    uint8_t motor_index,
    asr_fc_motor_telemetry_t *telemetry);

#endif
