#include "asr_fc/platform/gd3000_protocol.h"

#include <assert.h>
#include <stddef.h>

static void test_read_only_commands(void) {
    for (uint8_t index = 0u; index < 4u; ++index) {
        uint8_t command = 0xffu;
        assert(asr_fc_gd3000_null_command(index, &command) ==
               ASR_FC_GD3000_OK);
        assert(command == index);
    }

    uint8_t command = 0xffu;
    assert(asr_fc_gd3000_null_command(4u, &command) ==
           ASR_FC_GD3000_INVALID_STATUS_REGISTER);
    assert(command == 0xffu);
    assert(asr_fc_gd3000_null_command(0u, NULL) ==
           ASR_FC_GD3000_NULL_POINTER);
}

static void test_pipelined_status_read_sequence(void) {
    for (uint8_t index = 0u; index < 4u; ++index) {
        asr_fc_gd3000_status_read_t sequence = {0xffu, 0xffu};
        assert(asr_fc_gd3000_status_read_sequence(index, &sequence) ==
               ASR_FC_GD3000_OK);
        assert(sequence.request == index);
        assert(sequence.flush == 0u);
    }

    asr_fc_gd3000_status_read_t invalid = {0xa5u, 0x5au};
    assert(asr_fc_gd3000_status_read_sequence(4u, &invalid) ==
           ASR_FC_GD3000_INVALID_STATUS_REGISTER);
    assert(asr_fc_gd3000_status_read_sequence(0u, NULL) ==
           ASR_FC_GD3000_NULL_POINTER);
}

static void test_reset_status(void) {
    asr_fc_gd3000_status0_t status;
    assert(asr_fc_gd3000_decode_status0(0x80u, &status) ==
           ASR_FC_GD3000_OK);
    assert(status.reset_event);
    assert(!status.overtemperature);
    assert(!status.desaturation);
    assert(!status.low_vls);
    assert(!status.overcurrent);
    assert(!status.phase_error);
    assert(!status.framing_error);
    assert(!status.write_error);
    assert(!asr_fc_gd3000_status0_has_fault(&status));
}

static void test_fault_decode(void) {
    asr_fc_gd3000_status0_t status;
    assert(asr_fc_gd3000_decode_status0(0x7fu, &status) ==
           ASR_FC_GD3000_OK);
    assert(status.overtemperature);
    assert(status.desaturation);
    assert(status.low_vls);
    assert(status.overcurrent);
    assert(status.phase_error);
    assert(status.framing_error);
    assert(status.write_error);
    assert(!status.reset_event);
    assert(asr_fc_gd3000_status0_has_fault(&status));
    assert(asr_fc_gd3000_status0_has_fault(NULL));
    assert(asr_fc_gd3000_decode_status0(0u, NULL) ==
           ASR_FC_GD3000_NULL_POINTER);
}

int main(void) {
    test_read_only_commands();
    test_pipelined_status_read_sequence();
    test_reset_status();
    test_fault_decode();
    return 0;
}
