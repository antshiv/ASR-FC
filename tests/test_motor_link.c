#include "asr_fc/protocol/motor_link.h"

#include <assert.h>
#include <stdio.h>

static void test_command_round_trip(void) {
    const asr_fc_motor_command_t input = {
        .motor_index = 2,
        .armed = true,
        .flags = 3,
        .throttle_q15 = 16384,
        .session_id = 0x12345678u,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t frame_size = 0;
    assert(asr_fc_motor_link_encode_command(
        42, &input, frame, sizeof(frame), &frame_size) == ASR_FC_MOTOR_LINK_OK);
    uint32_t sequence = 0;
    asr_fc_motor_command_t output;
    assert(asr_fc_motor_link_decode_command(
        frame, frame_size, &sequence, &output) == ASR_FC_MOTOR_LINK_OK);
    assert(sequence == 42);
    assert(output.motor_index == input.motor_index);
    assert(output.armed == input.armed);
    assert(output.throttle_q15 == input.throttle_q15);
    assert(output.session_id == input.session_id);
    assert(output.command_timeout_ms == input.command_timeout_ms);
}

static void test_corruption_fails_closed(void) {
    const asr_fc_motor_command_t command = {
        .motor_index = 0,
        .throttle_q15 = 1,
        .session_id = 7,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t frame_size = 0;
    assert(asr_fc_motor_link_encode_command(
        1, &command, frame, sizeof(frame), &frame_size) == ASR_FC_MOTOR_LINK_OK);
    frame[12] ^= 0x40u;
    uint32_t sequence = 0;
    asr_fc_motor_command_t output;
    assert(asr_fc_motor_link_decode_command(
        frame, frame_size, &sequence, &output) ==
        ASR_FC_MOTOR_LINK_CRC_FAILURE);
    assert(!output.armed);
    assert(output.throttle_q15 == 0u);
}

static void test_telemetry_round_trip(void) {
    const asr_fc_motor_telemetry_t input = {
        .motor_index = 1,
        .motor_mode = 3,
        .fault_flags = 0x22u,
        .bus_voltage_mv = 24000,
        .dc_current_ma = -150,
        .electrical_period_us = 1200,
        .duty_q15 = 10000,
        .acknowledged_sequence = 99,
    };
    uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t frame_size = 0;
    assert(asr_fc_motor_link_encode_telemetry(
        100, &input, frame, sizeof(frame), &frame_size) == ASR_FC_MOTOR_LINK_OK);
    uint32_t sequence = 0;
    asr_fc_motor_telemetry_t output;
    assert(asr_fc_motor_link_decode_telemetry(
        frame, frame_size, &sequence, &output) == ASR_FC_MOTOR_LINK_OK);
    assert(sequence == 100);
    assert(output.fault_flags == input.fault_flags);
    assert(output.dc_current_ma == input.dc_current_ma);
    assert(output.duty_q15 == input.duty_q15);
    assert(output.acknowledged_sequence == input.acknowledged_sequence);
}

int main(void) {
    test_command_round_trip();
    test_corruption_fails_closed();
    test_telemetry_round_trip();
    puts("ASR-FC motor-link tests passed");
    return 0;
}
