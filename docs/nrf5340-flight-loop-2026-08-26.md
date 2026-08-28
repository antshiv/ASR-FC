# nRF5340 flight-loop evidence: 2026-08-26

## Purpose

This gate answers one narrow question: can the nRF5340 application core run
the current ASR attitude-stabilization path with enough margin to continue the
flight-controller work?

The image is motor-disabled. It contains no PWM, ESC, or motor-link driver.
Its input has the same shape as a decoded CEVA FSM300 sample, but the values
come from a deterministic rigid-body truth plant rather than the live sensor.

## Loop under test

```text
dynamic_models rigid-body state
        |
CEVA-shaped quaternion + calibrated rates
        |
ASR freshness, sequence, timing, and arming gates
        |
controlSystems checked quaternion PID
        |
controlSystems geometry mixer
        |
four virtual rotor speeds
        |
dynamic_models checked RK4 vehicle step
        +---------------- feedback ----------------+
```

The virtual aircraft begins with a 15-degree roll disturbance. The guidance
target is level attitude with hover collective thrust. The test executes 1,000
steps at 100 Hz, representing 10 seconds of simulated flight.

## Physical-board result

Board: Nordic nRF5340 DK application core

SDK used for this gate: NCS 2.4.2 / Zephyr 3.3.99

```text
ASR_FC_SIL steps=1000 initial_error_mdeg=15000 final_error_mdeg=58
final_rate_urad_s=38 control_avg_ns=561093 control_worst_ns=579515
loop_avg_ns=3447359 loop_worst_ns=3532000 motor_outputs_disabled=1
```

| Measurement | Result |
|---|---:|
| Initial attitude error | 15.000 deg |
| Final attitude error | 0.058 deg |
| Final roll rate | 38 urad/s |
| Controller + mixer average | 0.561 ms |
| Controller + mixer worst observed | 0.580 ms |
| Controller + RK4 plant average | 3.447 ms |
| Controller + RK4 plant worst observed | 3.532 ms |
| Flash | 56,212 B / 1 MiB (5.36%) |
| RAM | 17,656 B / 448 KiB (3.85%) |

At 100 Hz, the real flight path currently consumes under 6% of the 10 ms
period before CEVA transport, guidance, telemetry, and optional aiding are
added. The RK4 truth plant is a SIL instrument and will not run inside the
aircraft's production control loop.

## What this proves

- The checked quaternion PID and physically scaled mixer execute on Cortex-M33.
- The same rotor geometry drives control allocation and rigid-body response.
- A deterministic disturbance converges rather than diverges in this fixture.
- Current flash, RAM, and compute use leave room for live CEVA transport and
  additional flight-state logic.

## What this does not prove

- Live CEVA timing, axis convention, mounting transform, or dropout behavior.
- Stability with measured vehicle inertia, rotor coefficients, delay, noise,
  vibration, saturation, or motor dynamics.
- Barometer or ToF fusion. The optional aiding callback remains inactive.
- Safe motor operation or flight readiness.

## Reproduction

```bash
source /home/antshiv/ncs/toolchains/1f9b40e71a/env.sh
cd /data/Workspace/ASR-FC
west build -p always -b nrf5340dk_nrf5340_cpuapp \
  platforms/nordic/nrf5340/flight_core_benchmark \
  -d build/flight-core-benchmark
west flash -d build/flight-core-benchmark
```

The existing application flash was read back before the hardware run and
restored afterward. Evidence and the readback are stored on W530 under
`/data/Workspace/hardware-backups/20260826-asr-flight-core/`.
