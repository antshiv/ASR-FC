#!/usr/bin/env sh
set -eu

output=${1:?usage: capture_runner_evidence.sh OUTPUT_FILE}
mkdir -p "$(dirname "$output")"

{
    printf 'captured_utc=%s\n' "$(date -u +%Y-%m-%dT%H:%M:%SZ)"
    printf 'hostname=%s\n' "$(hostname 2>/dev/null || printf unknown)"
    printf 'kernel=%s\n' "$(uname -srmo 2>/dev/null || printf unknown)"
    printf '\n[lscpu]\n'
    if command -v lscpu >/dev/null 2>&1; then
        lscpu
    else
        cat /proc/cpuinfo 2>/dev/null || true
    fi
    printf '\n[memory]\n'
    if command -v free >/dev/null 2>&1; then
        free -h
    else
        cat /proc/meminfo 2>/dev/null || true
    fi
} >"$output"
