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
