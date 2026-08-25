# Flight-controller integration boundary

The nRF5340 owns estimation, flight modes, control, mixing, arming, and aircraft
failsafes. An NXP motor node owns commutation, current protection, pre-driver
faults, and motor telemetry. Neither side may infer that a missing message means
"keep the previous output."

```text
CEVA + sensors
      |
 nRF5340 flight controller
 controller -> mixer -> arming/failsafe
      |
 versioned motor-link command
      |
 KV31F/KV5x/RT1176 motor node
 six-step controller -> motor HAL -> GD3000
      |
 acknowledged telemetry + faults
```

## Validation progression

1. Native SIL encodes, transports, corrupts, and decodes frames in memory.
2. A four-channel virtual motor bank validates independent mixer outputs,
   sequence ownership, session establishment, command timeout, and telemetry.
3. W530 HIL routes USB CDC frames between both boards without motor power. One
   selected channel maps to the physical KV31F while three remain simulated.
4. A direct 3.3 V board link replaces the W530 routing while retaining the ABI.
5. GD3000 logic/status testing uses a current-limited supply and no motor.
6. Restrained motor testing begins only after timeout and fault injection pass.

The W530 USB bridge is a validation transport, not the production flight link.
The eventual physical transport can be UART, SPI, or an external CAN-FD
controller without changing the encoded command and telemetry contracts.

The stream parser consumes one byte at a time and does not expose a command
until the complete versioned frame, bounded payload length, and CRC pass. UART
noise, partial writes, and a corrupted frame therefore cannot reuse a previous
throttle value through the decoder.

## Own ESC path versus hobby PWM

The nRF5340 can retain four conventional PWM outputs as a fallback for
commercial hobby ESCs. That interface expresses a requested motor level through
a pulse width; it does not give the flight controller direct ownership of
commutation, current limiting, BEMF timing, or pre-driver faults.

The Antshiv ESC path instead sends four independent Q15 motor requests over the
motor link. The NXP node converts each accepted request into its own high-rate
motor-control state. A quad-ESC is therefore one coordinated assembly containing
four independent three-phase inverter channels, not one inverter electrically
shared by four motors.

## W530 USB bridge

`asr_fc_hil_usb_bridge` is protocol-aware. It validates both directions and
refuses armed or stale commands before forwarding bytes to the KV31F. Run it
only after identifying stable `/dev/serial/by-id` paths for both boards:

```bash
cmake -S . -B build
cmake --build build --parallel
./build/asr_fc_hil_usb_bridge NRF_TTY KV31F_TTY
```

The first KV31F sink still contains no PWM or pre-driver code. Successful HIL
at this stage means command routing, acknowledgement, corruption recovery, and
timeout behavior work. It does not mean a motor has been controlled.

## NXP four-motor reference

The downloaded AN5169 software is a complete four-motor sensorless BLDC
reference for the KV46. It contains separate `m1` through `m4` configuration
and state-machine modules, shared BLDC control, startup, commutation,
zero-crossing, pre-driver handling, and a prebuilt IAR image. Some mathematical
components are supplied as NXP binary libraries.

The separate Kinetis KV5x drone design archive contains the PCB Gerbers, not
the source code. ASR-FC therefore uses AN5169 as behavioral reference evidence;
it does not copy the application wholesale or represent it as KV31F/KV5x/
RT1176-portable code. Portable motor contracts belong in `motorDynamics`, while
each MCU and GD3000 implementation remains under `platforms/nxp`.
