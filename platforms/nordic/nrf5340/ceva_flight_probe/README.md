# nRF5340 live CEVA flight probe

This image joins live FSM300 game-rotation-vector and calibrated-gyroscope
reports, validates their age and timestamp skew, and feeds complete samples to
the portable flight core. The initial measured attitude becomes the dry-run
setpoint; moving the board produces virtual mixer commands in the serial log.

Sensor time and nRF host time are separate contracts. CEVA timestamps order
reports, bound quaternion/gyro skew, and determine controller `dt`. A 64-bit
nRF uptime timestamp records when each report arrived and determines
freshness. The flight path never subtracts one clock domain from the other.

The image cannot energize hardware. Its devicetree has no PWM node, its build
contains no ESC or motor-link implementation, and a compile-time assertion
keeps `ASR_FC_PHYSICAL_OUTPUTS_ENABLED` at zero. "Armed" flight-core state in
this probe means mathematical execution only.

The wiring matches the existing BLEDrone FSM300 prototype:

| Signal | nRF5340 DK pin |
|---|---:|
| FSM300 SCK | P0.06 |
| FSM300 MOSI | P0.07 |
| FSM300 MISO | P0.25 |
| FSM300 CSN | P0.26 |
| FSM300 INTN | P0.04 |
| FSM300 RSTN | P0.05 |
| FSM300 BOOTN | P1.10 |
| FSM300 WAKEN | P1.11 |

Build from an initialized nRF Connect SDK 3.4.0 shell:

```bash
west build -b nrf5340dk/nrf5340/cpuapp \
  platforms/nordic/nrf5340/ceva_flight_probe
```

nRF Connect SDK 2.4.2 uses the legacy board target
`nrf5340dk_nrf5340_cpuapp`. Keep the SDK and board target paired; do not infer
one target spelling works across both Zephyr generations.

Do not flash over a working board image until its binary and serial evidence
have been archived.
