#include "afc/sil/virtual_motor_bank.h"

#include <assert.h>
#include <stdio.h>

static afc_motor_command_t command(uint8_t index, uint16_t throttle) {
    const afc_motor_command_t value = {
        .motor_index = index,
        .throttle_q15 = throttle,
        .session_id = 11,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    return value;
}

static void test_four_channels_are_independent(void) {
    afc_virtual_motor_bank_t bank;
    afc_virtual_motor_bank_init(&bank);
    for (uint8_t index = 0; index < AFC_MOTOR_COUNT; ++index) {
        afc_motor_command_t value = command(index, 0);
        assert(afc_virtual_motor_bank_apply(
            &bank, (uint32_t)index + 1u, &value) == AFC_MOTOR_LINK_OK);
        value.armed = true;
        value.throttle_q15 = (uint16_t)(4000u * ((uint16_t)index + 1u));
        assert(afc_virtual_motor_bank_apply(
            &bank, (uint32_t)index + 10u, &value) == AFC_MOTOR_LINK_OK);
    }
    for (uint8_t index = 0; index < AFC_MOTOR_COUNT; ++index) {
        afc_motor_telemetry_t telemetry;
        assert(afc_virtual_motor_bank_telemetry(
            &bank, index, &telemetry) == AFC_MOTOR_LINK_OK);
        assert(telemetry.motor_mode == 1u);
        assert(telemetry.duty_q15 ==
               (uint16_t)(4000u * ((uint16_t)index + 1u)));
    }
}

static void test_timeout_disarms_only_stale_motor(void) {
    afc_virtual_motor_bank_t bank;
    afc_virtual_motor_bank_init(&bank);
    for (uint8_t index = 0; index < 2u; ++index) {
        afc_motor_command_t value = command(index, 0);
        assert(afc_virtual_motor_bank_apply(
            &bank, 1, &value) == AFC_MOTOR_LINK_OK);
        value.armed = true;
        value.throttle_q15 = 8000;
        assert(afc_virtual_motor_bank_apply(
            &bank, 2, &value) == AFC_MOTOR_LINK_OK);
    }
    afc_virtual_motor_bank_step(&bank, 15);
    afc_motor_command_t refresh = command(1, 8000);
    refresh.armed = true;
    assert(afc_virtual_motor_bank_apply(&bank, 3, &refresh) ==
           AFC_MOTOR_LINK_OK);
    afc_virtual_motor_bank_step(&bank, 10);
    assert(!bank.motor[0].armed);
    assert(bank.motor[1].armed);
    assert((bank.motor[0].fault_flags &
            AFC_VIRTUAL_MOTOR_FAULT_COMMAND_TIMEOUT) != 0u);
}

int main(void) {
    test_four_channels_are_independent();
    test_timeout_disarms_only_stale_motor();
    puts("AntshivFlightController virtual motor-bank tests passed");
    return 0;
}
