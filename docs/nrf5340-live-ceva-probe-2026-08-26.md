# nRF5340 live CEVA flight probe - 2026-08-26

## Scope

This gate replaced simulated CEVA-shaped frames with live FSM300
`GAME_ROTATION_VECTOR` and `GYROSCOPE_CALIBRATED` reports. The physical
nRF5340 DK joined the asynchronous reports, validated their timestamps, and
ran complete samples through the portable attitude controller and mixer.

This was a dry run. The firmware contained no PWM node, ESC link, or motor
driver. Its startup and diagnostic records reported `physical_outputs=0`.

## Hardware and software

- nRF5340 DK application core
- CEVA FSM300 over SPI at 500 kHz
- nRF Connect SDK 2.4.2 / Zephyr 3.3.99
- CEVA `sh2` pinned as the `modules/ceva_sh2` Git submodule
- 25 Hz game rotation vector and calibrated gyroscope reports
- ASR-FC CEVA assembler, quaternion PID, geometry mixer, and safety checks

The image used 46,896 bytes of flash (4.47%) and 20,560 bytes of RAM (4.48%).

## Evidence

After reset, the serial output included:

```text
ASR_FC_CEVA_PROBE boot physical_outputs=0 pwm=absent esc=absent
ASR_FC_CEVA_PROBE reports_enabled period_us=40000
ASR_FC_CEVA sample=25 step=0 status=3 ... physical_outputs=0
ASR_FC_CEVA_HEALTH decoded=112 complete=56 rejected=0 physical_outputs=0
```

The measured quaternion was normalized, the calibrated angular-rate values
changed with physical movement, and the four virtual Q15 mixer commands
changed independently. `step=0` is `ASR_FC_STEP_OK`; CEVA status 3 is the
highest SH-2 report status.

This proves the live sensor-to-controller software path. It does not prove
axis correctness, estimator accuracy, controller tuning, ESC timing, motor
control, or flight readiness.

## Clock-domain hardening - 2026-08-31

The original probe used the 32-bit nRF cycle counter as host time and compared
it directly with CEVA timestamps. At the application-core clock rate, that
counter wraps in about 33.5 seconds; the two timestamps also do not share an
origin. The sample contract now carries CEVA sensor time for ordering and
controller `dt`, plus 64-bit nRF uptime at receipt for freshness and timeouts.

The corrected image was rebuilt with nRF Connect SDK 2.4.2, erased, flashed,
verified, and reset. A 65-second serial capture continued beyond the previous
wrap boundary and reached:

```text
ASR_FC_CEVA_HEALTH decoded=3604 complete=1802 rejected=0 physical_outputs=0
```

Every sampled controller result in the capture remained `step=0`. The image
used 47,264 bytes of flash (4.51%) and 20,576 bytes of RAM (4.49%).

## nRF Connect SDK 3.4.0 validation - 2026-08-31

The latest stable Nordic SDK and its matching toolchain were installed
side-by-side with 2.4.2. The same motor-disabled source built against nRF
Connect SDK 3.4.0, Zephyr 4.4.0, West 1.5.0, GCC 14.3.0, and Zephyr SDK 1.0.1.
The qualified board target changed to `nrf5340dk/nrf5340/cpuapp`.

```bash
nrfutil sdk-manager toolchain launch --ncs-version v3.4.0 -- \
  west build --pristine \
  -b nrf5340dk/nrf5340/cpuapp \
  platforms/nordic/nrf5340/ceva_flight_probe
```

The 3.4.0 image used 53,332 bytes of flash (5.09%) and 20,488 bytes of RAM
(4.47%). It was erased, programmed, verified, reset, and exercised with the
physical FSM300. A clean post-reset trace crossed the former cycle-counter
wrap boundary and reached:

```text
ASR_FC_CEVA_HEALTH decoded=1862 complete=931 rejected=0 physical_outputs=0
```

All sampled controller results remained `step=0`. Zephyr 4.4.0 warns that it
selects Picolibc rather than the probe's requested Newlib configuration. The
image builds and runs, but libc selection remains an explicit follow-up rather
than being silently changed in this validation.

## Reproduction

```bash
export LD_LIBRARY_PATH=/home/antshiv/ncs/toolchains/1f9b40e71a/usr/local/lib
source /home/antshiv/ncs/v2.4.2/zephyr/zephyr-env.sh
/home/antshiv/anaconda3/bin/west build -p always \
  -b nrf5340dk_nrf5340_cpuapp \
  platforms/nordic/nrf5340/ceva_flight_probe \
  -d build/ceva-flight-probe -- \
  -DCMAKE_MAKE_PROGRAM=/usr/bin/ninja
```

The W530's west runner currently lacks a Python runner dependency. The
verified image was therefore flashed directly with the installed Nordic tool:

```bash
nrfjprog --program build/ceva-flight-probe/zephyr/zephyr.hex \
  --sectorerase --verify --reset --family NRF53 \
  --coprocessor CP_APPLICATION --snr 960169267
```

## Next gate

1. Record controlled roll, pitch, and yaw motions separately.
2. Verify the FSM300-to-airframe axis transform and quaternion convention.
3. Inject stale, repeated, skewed, and invalid reports on hardware.
4. Confirm that each fault produces a latched safe state.
5. Keep all physical outputs absent until those checks pass.
