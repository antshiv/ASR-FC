#include "asr_fc/sensors/ceva_fsm300_adapter.h"

#include <assert.h>
#include <math.h>
#include <stdio.h>

static asr_fc_ceva_adapter_t make_adapter(void) {
    asr_fc_ceva_adapter_t adapter;
    const asr_fc_ceva_adapter_config_t config = {
        .max_component_age_us = 30000u,
        .max_component_skew_us = 5000u,
        .minimum_quaternion_norm = 0.90,
        .maximum_quaternion_norm = 1.10,
    };
    assert(asr_fc_ceva_adapter_init(&adapter, &config) ==
           ASR_FC_CEVA_ADAPTER_OK);
    return adapter;
}

static void test_complete_frame(void) {
    asr_fc_ceva_adapter_t adapter = make_adapter();
    const double quaternion[4] = {2.0, 0.0, 0.0, 0.0};
    const double invalid_norm[4] = {0.0, 0.0, 0.0, 0.0};
    const double unit_quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    const double gyro[3] = {0.1, -0.2, 0.3};
    asr_fc_ceva_sample_t sample;

    assert(asr_fc_ceva_adapter_update_rotation(&adapter, quaternion, 3u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_INVALID_VALUE);
    assert(asr_fc_ceva_adapter_update_rotation(&adapter, invalid_norm, 3u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_INVALID_VALUE);
    assert(asr_fc_ceva_adapter_update_rotation(&adapter, unit_quaternion, 3u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 10000u, &sample) ==
           ASR_FC_CEVA_ADAPTER_INCOMPLETE);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 12000u, 12000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 12500u, &sample) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(sample.sequence == 1u);
    assert(sample.timestamp_us == 12000u);
    assert(sample.accuracy == 3u);
    assert(sample.attitude_valid);
    assert(fabs(sample.quaternion[0] - 1.0) < 1e-12);
    assert(fabs(sample.angular_rate[1] + 0.2) < 1e-12);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 13000u, &sample) ==
           ASR_FC_CEVA_ADAPTER_INCOMPLETE);
}

static void test_stale_and_skewed_components(void) {
    asr_fc_ceva_adapter_t adapter = make_adapter();
    const double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    const double gyro[3] = {0.0, 0.0, 0.0};
    asr_fc_ceva_sample_t sample;

    assert(asr_fc_ceva_adapter_update_rotation(&adapter, quaternion, 2u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 17000u, 17000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 18000u, &sample) ==
           ASR_FC_CEVA_ADAPTER_COMPONENT_SKEW);

    adapter = make_adapter();
    assert(asr_fc_ceva_adapter_update_rotation(&adapter, quaternion, 2u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 12000u, 12000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 50001u, &sample) ==
           ASR_FC_CEVA_ADAPTER_STALE);
}

static void test_repeated_reports_and_sign_continuity(void) {
    asr_fc_ceva_adapter_t adapter = make_adapter();
    const double positive[4] = {1.0, 0.0, 0.0, 0.0};
    const double negative[4] = {-1.0, 0.0, 0.0, 0.0};
    const double gyro[3] = {0.0, 0.0, 0.0};
    asr_fc_ceva_sample_t sample;

    assert(asr_fc_ceva_adapter_update_rotation(&adapter, positive, 3u,
                                                10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 10000u, 10000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 10100u, &sample) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_rotation(&adapter, positive, 3u,
                                                10000u, 10100u) ==
           ASR_FC_CEVA_ADAPTER_REPEATED_REPORT);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 10000u, 10100u) ==
           ASR_FC_CEVA_ADAPTER_REPEATED_REPORT);

    assert(asr_fc_ceva_adapter_update_rotation(&adapter, negative, 3u,
                                                20000u, 20000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(&adapter, gyro, 20000u, 20000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 20100u, &sample) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(sample.sequence == 2u);
    assert(sample.quaternion[0] > 0.0);
}

static void test_independent_sensor_and_host_clocks(void) {
    asr_fc_ceva_adapter_t adapter = make_adapter();
    const double quaternion[4] = {1.0, 0.0, 0.0, 0.0};
    const double gyro[3] = {0.0, 0.0, 0.0};
    asr_fc_ceva_sample_t sample;
    const uint64_t host_after_32_bit_cycle_wrap = 4300000000ull;

    assert(asr_fc_ceva_adapter_update_rotation(
               &adapter, quaternion, 3u, 5000000u,
               host_after_32_bit_cycle_wrap) == ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(
               &adapter, gyro, 5001000u,
               host_after_32_bit_cycle_wrap + 100u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(
               &adapter, host_after_32_bit_cycle_wrap + 200u, &sample) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(sample.timestamp_us == 5001000u);
    assert(sample.received_at_us == host_after_32_bit_cycle_wrap + 100u);

    adapter = make_adapter();
    assert(asr_fc_ceva_adapter_update_rotation(
               &adapter, quaternion, 3u, 7000000u, 100000u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_update_gyro(
               &adapter, gyro, 7001000u, 100100u) ==
           ASR_FC_CEVA_ADAPTER_OK);
    assert(asr_fc_ceva_adapter_take_sample(&adapter, 130101u, &sample) ==
           ASR_FC_CEVA_ADAPTER_STALE);
}

int main(void) {
    test_complete_frame();
    test_stale_and_skewed_components();
    test_repeated_reports_and_sign_continuity();
    test_independent_sensor_and_host_clocks();
    puts("ceva_fsm300_adapter_tests: PASS");
    return 0;
}
