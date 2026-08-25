# KV31F HIL motor sink

This image decodes ASR-FC command frames for motor zero and acknowledges them
with telemetry. It deliberately does not compile or initialize FTM/PWM, ADC,
DSPI, GD3000, or gate-output code. Telemetry always reports the
`OUTPUT_INHIBITED` fault and zero duty.

It is the first safe target for USB HIL transport validation. It must not be
extended to produce PWM until reset, timeout, corruption, and stale-command
tests pass on hardware.
