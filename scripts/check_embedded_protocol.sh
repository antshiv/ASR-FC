#!/usr/bin/env sh
set -eu

cc="${ARM_CC:-arm-none-eabi-gcc}"
out_dir="${TMPDIR:-/tmp}/asr-fc-embedded"
mkdir -p "$out_dir"

for cpu in cortex-m4 cortex-m33; do
    for source in application/protocol/motor_link.c \
                  application/sil/virtual_motor_bank.c; do
        object="$out_dir/${cpu}-$(basename "${source%.c}").o"
        "$cc" \
            -std=c11 \
            -mcpu="$cpu" \
            -mthumb \
            -mfloat-abi=soft \
            -ffreestanding \
            -fno-builtin \
            -Wall \
            -Wextra \
            -Wpedantic \
            -Wconversion \
            -Werror \
            -Iinclude \
            -c "$source" \
            -o "$object"
    done
done

printf 'ASR-FC protocol compiled for Cortex-M4 and Cortex-M33\n'
