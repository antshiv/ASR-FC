# Host HIL contract

ASR-FC now separates three validation paths:

1. Native SIL runs the controller, mixer, and plant on the host.
2. nRF5340 SIL runs the controller, mixer, and RK4 plant on the physical MCU.
3. Host HIL runs the plant on the host while the physical nRF5340 runs one
   controller and mixer step for each accepted sensor frame.

The host-HIL wire format is versioned, explicitly little-endian, CRC protected,
and encoded field by field. It does not transmit native C structs. Requests
carry a session, sequence, simulated sensor timestamp, attitude, rates,
acceleration, guidance, arming request, and collective thrust. Responses carry
the acknowledged sequence, MCU timestamp, execution time, flight state, fault
state, controller wrench, and four virtual motor commands.

Collective thrust is a non-negative magnitude on the wire. The flight core maps
that magnitude onto the common thrust-axis sign declared by the four rotor
records before invoking the geometry-derived mixer. The nRF5340 host endpoint
uses the same FRD, body-to-NED frame, rotor order, positions, thrust axes, spin
directions, and coefficients as the `asr-reference-quad` AeroDyn contract.

Host and MCU clocks are intentionally independent. The simulated sensor
timestamp determines controller `dt`; the MCU timestamp measures the physical
execution path. A session change resets the controller. Duplicate, corrupt, or
invalid input disarms or latches the checked failsafe path.

## Evidence on 2026-08-27

- Seven native test suites pass with AddressSanitizer and UndefinedBehaviorSanitizer.
- Endpoint tests cover session reset, arm/disarm, duplicate sequence, CRC failure,
  and virtual motor output.
- Nordic NCS 2.4.2 builds the endpoint for `nrf5340dk_nrf5340_cpuapp`.
- The image uses 49,352 bytes of flash and 16,016 bytes of RAM.
- The image compiles no PWM, GD3000, commutation, or physical motor-output source.

The next host integration should consume ASR-FC as a pinned dependency and call
this C contract. AeroDyn must not maintain a copied implementation of the wire
format. After the dependency is published, AeroDyn can send its RK4 plant state
and joystick-derived guidance, wait for the nRF response, apply the returned
motor speeds, and expose transport latency and fault state in its existing UI.
