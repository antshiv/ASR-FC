#ifndef AFC_SIL_VIRTUAL_MOTOR_BANK_H
#define AFC_SIL_VIRTUAL_MOTOR_BANK_H

#include <stdbool.h>
#include <stdint.h>

#include "afc/protocol/motor_link.h"

#define AFC_MOTOR_COUNT 4u
#define AFC_VIRTUAL_MOTOR_FAULT_COMMAND_TIMEOUT (1u << 0)
#define AFC_VIRTUAL_MOTOR_FAULT_SEQUENCE (1u << 1)
#define AFC_VIRTUAL_MOTOR_FAULT_SESSION (1u << 2)

typedef struct {
    bool initialized;
    bool armed;
    uint16_t throttle_q15;
    uint16_t command_timeout_ms;
    uint32_t elapsed_ms;
    uint32_t session_id;
    uint32_t last_sequence;
    uint32_t fault_flags;
} afc_virtual_motor_t;

typedef struct {
    afc_virtual_motor_t motor[AFC_MOTOR_COUNT];
} afc_virtual_motor_bank_t;

void afc_virtual_motor_bank_init(afc_virtual_motor_bank_t *bank);

afc_motor_link_status_t afc_virtual_motor_bank_apply(
    afc_virtual_motor_bank_t *bank,
    uint32_t sequence,
    const afc_motor_command_t *command);

void afc_virtual_motor_bank_step(
    afc_virtual_motor_bank_t *bank,
    uint32_t elapsed_ms);

afc_motor_link_status_t afc_virtual_motor_bank_telemetry(
    const afc_virtual_motor_bank_t *bank,
    uint8_t motor_index,
    afc_motor_telemetry_t *telemetry);

#endif
