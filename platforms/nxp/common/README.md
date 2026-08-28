# NXP motor-node common code

This directory contains portable protocol policy shared by the KV31F, KV5x,
and RT1176 hardware adapters. It does not configure MCU pins or enable a power
stage.

`gd3000_protocol.c` initially exposes only the four read-only `NULL` commands
defined by the GD3000 data sheet and decoding for status register 0. It
deliberately does not expose register writes, fault clearing, dead-time
calibration, `EN1`/`EN2`, PWM, or gate commands. Those operations remain behind
later hardware gates.
