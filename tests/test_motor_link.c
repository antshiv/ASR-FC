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

static void test_stream_parser_handles_noise_and_fragmentation(void) {
    const asr_fc_motor_command_t command = {
        .motor_index = 3,
        .throttle_q15 = 1234,
        .session_id = 77,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    uint8_t encoded[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t encoded_size = 0;
    assert(asr_fc_motor_link_encode_command(
        9, &command, encoded, sizeof(encoded), &encoded_size) ==
        ASR_FC_MOTOR_LINK_OK);

    asr_fc_motor_link_parser_t parser;
    asr_fc_motor_link_parser_init(&parser);
    uint8_t decoded[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t decoded_size = 0;
    bool ready = false;
    const uint8_t noise[] = {0x00u, 0xffu, (uint8_t)'A', 0x01u};
    for (size_t index = 0; index < sizeof(noise); ++index) {
        assert(asr_fc_motor_link_parser_push(
            &parser, noise[index], decoded, sizeof(decoded),
            &decoded_size, &ready) == ASR_FC_MOTOR_LINK_OK);
        assert(!ready);
    }
    for (size_t index = 0; index < encoded_size; ++index) {
        assert(asr_fc_motor_link_parser_push(
            &parser, encoded[index], decoded, sizeof(decoded),
            &decoded_size, &ready) == ASR_FC_MOTOR_LINK_OK);
        assert(ready == (index + 1u == encoded_size));
    }
    assert(decoded_size == encoded_size);
    uint32_t sequence = 0;
    asr_fc_motor_command_t output;
    assert(asr_fc_motor_link_decode_command(
        decoded, decoded_size, &sequence, &output) == ASR_FC_MOTOR_LINK_OK);
    assert(sequence == 9u);
    assert(output.motor_index == 3u);
}

static void test_stream_parser_rejects_corruption_and_recovers(void) {
    const asr_fc_motor_command_t command = {
        .motor_index = 0,
        .throttle_q15 = 100,
        .session_id = 12,
        .command_timeout_ms = 20,
        .heartbeat_timeout_ms = 100,
    };
    uint8_t encoded[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t encoded_size = 0;
    assert(asr_fc_motor_link_encode_command(
        3, &command, encoded, sizeof(encoded), &encoded_size) ==
        ASR_FC_MOTOR_LINK_OK);
    encoded[11] ^= 0x01u;

    asr_fc_motor_link_parser_t parser;
    asr_fc_motor_link_parser_init(&parser);
    uint8_t decoded[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
    size_t decoded_size = 0;
    bool ready = false;
    for (size_t index = 0; index < encoded_size; ++index) {
        const asr_fc_motor_link_status_t status =
            asr_fc_motor_link_parser_push(
                &parser, encoded[index], decoded, sizeof(decoded),
                &decoded_size, &ready);
        if (index + 1u == encoded_size) {
            assert(status == ASR_FC_MOTOR_LINK_CRC_FAILURE);
        } else {
            assert(status == ASR_FC_MOTOR_LINK_OK);
        }
        assert(!ready);
    }

    encoded[11] ^= 0x01u;
    for (size_t index = 0; index < encoded_size; ++index) {
        assert(asr_fc_motor_link_parser_push(
            &parser, encoded[index], decoded, sizeof(decoded),
            &decoded_size, &ready) == ASR_FC_MOTOR_LINK_OK);
    }
    assert(ready);
}

int main(void) {
    test_command_round_trip();
    test_corruption_fails_closed();
    test_telemetry_round_trip();
    test_stream_parser_handles_noise_and_fragmentation();
    test_stream_parser_rejects_corruption_and_recovers();
    puts("ASR-FC motor-link tests passed");
    return 0;
}
