# Disarmed nRF5340 to KV31F HIL bring-up

The first physical motor-link test used an nRF5340 DK, a W530 USB bridge, and
an FRDM-KV31F. Both original MCU flash images were read back before testing.

The nRF image emitted four independently addressed Q15 commands. The host
bridge accepted only complete version-1 frames with valid CRCs, rejected armed
and stale commands, and forwarded accepted commands to the KV31F. The KV31F
image acknowledged motor zero while reporting `OUTPUT_INHIBITED` and zero duty.
It contained no FTM/PWM, ADC, DSPI, GD3000, or gate-output initialization.

The first four-second capture produced:

| Evidence | Result |
|---|---:|
| Validated commands forwarded | 541 |
| Telemetry acknowledgements | 134 |
| Armed commands rejected | 0 |
| Stale commands rejected | 0 |
| Invalid frames | 1 |
| KV31F reported duty | 0 |
| KV31F fault state | `0x80000000` (`OUTPUT_INHIBITED`) |

The one invalid frame was the expected partial frame observed when the bridge
attached to an already-running byte stream. The stream parser resynchronized
and all subsequent frames decoded.

The capture also exposed a scheduling error. Serializing four 28-byte frames at
115200 baud takes approximately 10 ms. Sleeping 20 ms only after transmission
therefore released each motor command at roughly 33 Hz rather than the intended
50 Hz. The command source was changed to use a fixed 20 ms release schedule so
serialization time is included in, rather than added to, the period.

The repeated four-second capture after that correction produced:

| Evidence | Result |
|---|---:|
| Validated commands forwarded | 799 |
| Motor-zero acknowledgements | 199 |
| Approximate aggregate command rate | 200 commands/s |
| Approximate selected-motor rate | 50 commands/s |
| Armed commands rejected | 0 |
| Stale commands rejected | 0 |
| Invalid frames | 0 |
| KV31F reported duty | 0 |
| KV31F fault state | `0x80000000` (`OUTPUT_INHIBITED`) |

This gate proves the framed digital boundary and output inhibition. It does not
prove commutation, GD3000 operation, motor control, or flight readiness.
