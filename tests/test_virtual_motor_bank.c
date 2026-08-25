#include "asr_fc/sil/virtual_motor_bank.h"

#include <assert.h>
#include <stdio.h>

static asr_fc_motor_command_t command(uint8_t index, uint16_t throttle) {
    const asr_fc_motor_command_t value = {
        .motor_index = index,
        .throttle_q15 = throttle,
        .session_id = 11,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    return value;
}

static void test_four_channels_are_independent(void) {
    asr_fc_virtual_motor_bank_t bank;
    asr_fc_virtual_motor_bank_init(&bank);
    for (uint8_t index = 0; index < ASR_FC_MOTOR_COUNT; ++index) {
        asr_fc_motor_command_t value = command(index, 0);
        assert(asr_fc_virtual_motor_bank_apply(
            &bank, (uint32_t)index + 1u, &value) == ASR_FC_MOTOR_LINK_OK);
        value.armed = true;
        value.throttle_q15 = (uint16_t)(4000u * ((uint16_t)index + 1u));
        assert(asr_fc_virtual_motor_bank_apply(
            &bank, (uint32_t)index + 10u, &value) == ASR_FC_MOTOR_LINK_OK);
    }
    for (uint8_t index = 0; index < ASR_FC_MOTOR_COUNT; ++index) {
        asr_fc_motor_telemetry_t telemetry;
        assert(asr_fc_virtual_motor_bank_telemetry(
            &bank, index, &telemetry) == ASR_FC_MOTOR_LINK_OK);
        assert(telemetry.motor_mode == 1u);
        assert(telemetry.duty_q15 ==
               (uint16_t)(4000u * ((uint16_t)index + 1u)));
    }
}

static void test_timeout_disarms_only_stale_motor(void) {
    asr_fc_virtual_motor_bank_t bank;
    asr_fc_virtual_motor_bank_init(&bank);
    for (uint8_t index = 0; index < 2u; ++index) {
        asr_fc_motor_command_t value = command(index, 0);
        assert(asr_fc_virtual_motor_bank_apply(
            &bank, 1, &value) == ASR_FC_MOTOR_LINK_OK);
        value.armed = true;
        value.throttle_q15 = 8000;
        assert(asr_fc_virtual_motor_bank_apply(
            &bank, 2, &value) == ASR_FC_MOTOR_LINK_OK);
    }
    asr_fc_virtual_motor_bank_step(&bank, 15);
    asr_fc_motor_command_t refresh = command(1, 8000);
    refresh.armed = true;
    assert(asr_fc_virtual_motor_bank_apply(&bank, 3, &refresh) ==
           ASR_FC_MOTOR_LINK_OK);
    asr_fc_virtual_motor_bank_step(&bank, 10);
    assert(!bank.motor[0].armed);
    assert(bank.motor[1].armed);
    assert((bank.motor[0].fault_flags &
            ASR_FC_VIRTUAL_MOTOR_FAULT_COMMAND_TIMEOUT) != 0u);
}

int main(void) {
    test_four_channels_are_independent();
    test_timeout_disarms_only_stale_motor();
    puts("ASR-FC virtual motor-bank tests passed");
    return 0;
}
