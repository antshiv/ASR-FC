#include "afc/sil/virtual_motor_bank.h"

#include <string.h>

void afc_virtual_motor_bank_init(afc_virtual_motor_bank_t *bank) {
    if (bank) {
        memset(bank, 0, sizeof(*bank));
    }
}

static void disarm(afc_virtual_motor_t *motor) {
    motor->armed = false;
    motor->throttle_q15 = 0u;
}

afc_motor_link_status_t afc_virtual_motor_bank_apply(
    afc_virtual_motor_bank_t *bank,
    uint32_t sequence,
    const afc_motor_command_t *command) {
    if (!bank || !command) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    if (command->motor_index >= AFC_MOTOR_COUNT ||
        command->throttle_q15 > 32767u || command->command_timeout_ms == 0u) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    afc_virtual_motor_t *motor = &bank->motor[command->motor_index];
    if (motor->initialized && sequence <= motor->last_sequence) {
        motor->fault_flags |= AFC_VIRTUAL_MOTOR_FAULT_SEQUENCE;
        disarm(motor);
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    if ((!motor->initialized || command->session_id != motor->session_id) &&
        command->armed) {
        motor->fault_flags |= AFC_VIRTUAL_MOTOR_FAULT_SESSION;
        disarm(motor);
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }

    motor->initialized = true;
    motor->session_id = command->session_id;
    motor->last_sequence = sequence;
    motor->command_timeout_ms = command->command_timeout_ms;
    motor->elapsed_ms = 0u;
    motor->armed = command->armed;
    motor->throttle_q15 = command->armed ? command->throttle_q15 : 0u;
    return AFC_MOTOR_LINK_OK;
}

void afc_virtual_motor_bank_step(
    afc_virtual_motor_bank_t *bank,
    uint32_t elapsed_ms) {
    if (!bank) {
        return;
    }
    for (uint32_t index = 0; index < AFC_MOTOR_COUNT; ++index) {
        afc_virtual_motor_t *motor = &bank->motor[index];
        if (!motor->initialized || !motor->armed) {
            continue;
        }
        if (UINT32_MAX - motor->elapsed_ms < elapsed_ms) {
            motor->elapsed_ms = UINT32_MAX;
        } else {
            motor->elapsed_ms += elapsed_ms;
        }
        if (motor->elapsed_ms > motor->command_timeout_ms) {
            motor->fault_flags |= AFC_VIRTUAL_MOTOR_FAULT_COMMAND_TIMEOUT;
            disarm(motor);
        }
    }
}

afc_motor_link_status_t afc_virtual_motor_bank_telemetry(
    const afc_virtual_motor_bank_t *bank,
    uint8_t motor_index,
    afc_motor_telemetry_t *telemetry) {
    if (!bank || !telemetry) {
        return AFC_MOTOR_LINK_NULL_POINTER;
    }
    if (motor_index >= AFC_MOTOR_COUNT) {
        return AFC_MOTOR_LINK_INVALID_FRAME;
    }
    const afc_virtual_motor_t *motor = &bank->motor[motor_index];
    memset(telemetry, 0, sizeof(*telemetry));
    telemetry->motor_index = motor_index;
    telemetry->motor_mode = motor->armed ? 1u : 0u;
    telemetry->fault_flags = motor->fault_flags;
    telemetry->duty_q15 = motor->throttle_q15;
    telemetry->acknowledged_sequence = motor->last_sequence;
    return AFC_MOTOR_LINK_OK;
}
