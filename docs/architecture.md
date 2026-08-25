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
