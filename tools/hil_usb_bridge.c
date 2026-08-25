#define _DEFAULT_SOURCE

#include "asr_fc/protocol/motor_link.h"

#include <errno.h>
#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <termios.h>
#include <unistd.h>

typedef struct {
    uint64_t commands_forwarded;
    uint64_t telemetry_forwarded;
    uint64_t armed_commands_rejected;
    uint64_t stale_commands_rejected;
    uint64_t invalid_frames;
} bridge_stats_t;

typedef struct {
    bool initialized;
    uint32_t session_id;
    uint32_t sequence;
} command_order_t;

static volatile sig_atomic_t running = 1;

static void stop_bridge(int signal_number) {
    (void)signal_number;
    running = 0;
}

static int configure_serial(const char *path) {
    const int descriptor = open(path, O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (descriptor < 0) {
        fprintf(stderr, "open %s: %s\n", path, strerror(errno));
        return -1;
    }

    struct termios settings;
    if (tcgetattr(descriptor, &settings) != 0) {
        fprintf(stderr, "tcgetattr %s: %s\n", path, strerror(errno));
        close(descriptor);
        return -1;
    }
    cfmakeraw(&settings);
    if (cfsetispeed(&settings, B115200) != 0 ||
        cfsetospeed(&settings, B115200) != 0) {
        fprintf(stderr, "baud %s: %s\n", path, strerror(errno));
        close(descriptor);
        return -1;
    }
    settings.c_cflag |= CLOCAL | CREAD;
    settings.c_cflag &= (tcflag_t)~CSTOPB;
    settings.c_cflag &= (tcflag_t)~CRTSCTS;
    if (tcsetattr(descriptor, TCSANOW, &settings) != 0) {
        fprintf(stderr, "tcsetattr %s: %s\n", path, strerror(errno));
        close(descriptor);
        return -1;
    }
    tcflush(descriptor, TCIOFLUSH);
    return descriptor;
}

static bool write_all(int descriptor, const uint8_t *data, size_t size) {
    size_t written = 0u;
    while (written < size) {
        const ssize_t result = write(descriptor, data + written,
            size - written);
        if (result > 0) {
            written += (size_t)result;
            continue;
        }
        if (result < 0 && (errno == EINTR || errno == EAGAIN)) {
            continue;
        }
        return false;
    }
    return true;
}

static bool sequence_is_new(command_order_t *order, uint32_t session_id,
    uint32_t sequence) {
    if (!order->initialized || order->session_id != session_id) {
        order->initialized = true;
        order->session_id = session_id;
        order->sequence = sequence;
        return true;
    }
    if ((int32_t)(sequence - order->sequence) <= 0) {
        return false;
    }
    order->sequence = sequence;
    return true;
}

static bool bridge_commands(int source, int sink,
    asr_fc_motor_link_parser_t *parser, command_order_t *order,
    bridge_stats_t *stats) {
    uint8_t input[256];
    const ssize_t received = read(source, input, sizeof(input));
    if (received < 0 && errno != EAGAIN && errno != EINTR) {
        return false;
    }
    for (ssize_t index = 0; index < received; ++index) {
        uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
        size_t frame_size = 0u;
        bool ready = false;
        const asr_fc_motor_link_status_t status =
            asr_fc_motor_link_parser_push(parser, input[index], frame,
                sizeof(frame), &frame_size, &ready);
        if (status != ASR_FC_MOTOR_LINK_OK) {
            ++stats->invalid_frames;
            continue;
        }
        if (!ready) {
            continue;
        }
        uint32_t sequence = 0u;
        asr_fc_motor_command_t command;
        if (asr_fc_motor_link_decode_command(frame, frame_size, &sequence,
                &command) != ASR_FC_MOTOR_LINK_OK) {
            ++stats->invalid_frames;
            continue;
        }
        if (command.armed) {
            ++stats->armed_commands_rejected;
            continue;
        }
        if (!sequence_is_new(order, command.session_id, sequence)) {
            ++stats->stale_commands_rejected;
            continue;
        }
        if (!write_all(sink, frame, frame_size)) {
            return false;
        }
        ++stats->commands_forwarded;
    }
    return true;
}

static bool bridge_telemetry(int sink, int source,
    asr_fc_motor_link_parser_t *parser, bridge_stats_t *stats) {
    uint8_t input[256];
    const ssize_t received = read(sink, input, sizeof(input));
    if (received < 0 && errno != EAGAIN && errno != EINTR) {
        return false;
    }
    for (ssize_t index = 0; index < received; ++index) {
        uint8_t frame[ASR_FC_MOTOR_LINK_MAX_FRAME_SIZE];
        size_t frame_size = 0u;
        bool ready = false;
        const asr_fc_motor_link_status_t status =
            asr_fc_motor_link_parser_push(parser, input[index], frame,
                sizeof(frame), &frame_size, &ready);
        if (status != ASR_FC_MOTOR_LINK_OK) {
            ++stats->invalid_frames;
            continue;
        }
        if (!ready) {
            continue;
        }
        uint32_t sequence = 0u;
        asr_fc_motor_telemetry_t telemetry;
        if (asr_fc_motor_link_decode_telemetry(frame, frame_size, &sequence,
                &telemetry) != ASR_FC_MOTOR_LINK_OK) {
            ++stats->invalid_frames;
            continue;
        }
        if (!write_all(source, frame, frame_size)) {
            return false;
        }
        ++stats->telemetry_forwarded;
        fprintf(stderr,
            "telemetry seq=%u motor=%u ack=%u faults=0x%08x duty=%u\n",
            sequence, telemetry.motor_index,
            telemetry.acknowledged_sequence, telemetry.fault_flags,
            telemetry.duty_q15);
    }
    return true;
}

int main(int argc, char **argv) {
    if (argc != 3) {
        fprintf(stderr, "usage: %s NRF_TTY KV31F_TTY\n", argv[0]);
        return 2;
    }
    const int source = configure_serial(argv[1]);
    if (source < 0) {
        return 1;
    }
    const int sink = configure_serial(argv[2]);
    if (sink < 0) {
        close(source);
        return 1;
    }

    signal(SIGINT, stop_bridge);
    signal(SIGTERM, stop_bridge);
    asr_fc_motor_link_parser_t command_parser;
    asr_fc_motor_link_parser_t telemetry_parser;
    asr_fc_motor_link_parser_init(&command_parser);
    asr_fc_motor_link_parser_init(&telemetry_parser);
    command_order_t order = {0};
    bridge_stats_t stats = {0};
    struct pollfd ports[2] = {
        {.fd = source, .events = POLLIN},
        {.fd = sink, .events = POLLIN},
    };

    fprintf(stderr, "ASR-FC HIL bridge: %s <-> %s (disarmed only)\n",
        argv[1], argv[2]);
    while (running) {
        const int poll_status = poll(ports, 2u, 250);
        if (poll_status < 0 && errno != EINTR) {
            break;
        }
        if ((ports[0].revents & POLLIN) != 0 &&
            !bridge_commands(source, sink, &command_parser, &order, &stats)) {
            break;
        }
        if ((ports[1].revents & POLLIN) != 0 &&
            !bridge_telemetry(sink, source, &telemetry_parser, &stats)) {
            break;
        }
        if ((ports[0].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0 ||
            (ports[1].revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
            break;
        }
    }

    fprintf(stderr,
        "commands=%llu telemetry=%llu armed_rejected=%llu "
        "stale_rejected=%llu invalid=%llu\n",
        (unsigned long long)stats.commands_forwarded,
        (unsigned long long)stats.telemetry_forwarded,
        (unsigned long long)stats.armed_commands_rejected,
        (unsigned long long)stats.stale_commands_rejected,
        (unsigned long long)stats.invalid_frames);
    close(sink);
    close(source);
    return 0;
}
