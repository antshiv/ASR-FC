# nRF5340 host HIL endpoint

This Zephyr image makes the physical nRF5340 execute one checked ASR-FC
controller and mixer step for each synthetic sensor-and-guidance frame received
from a host plant. It returns four virtual motor commands, controller output,
observed state, fault state, and measured execution time.

The endpoint is deliberately lockstep. Host and board clocks do not need to be
synchronized: the simulated sensor timestamp controls integration time while
the response reports the independent board timestamp. A changed session resets
the flight core. Duplicate, stale, corrupt, or invalid frames cannot retain an
armed state.

This image contains no PWM, GD3000, commutation, or physical motor-output
implementation. Its virtual outputs are suitable for advancing AeroDyn's RK4
plant and for measuring the real MCU control path before propulsion is enabled.

Build with the same Nordic Connect SDK environment used by the CEVA probe:

```bash
west build -b nrf5340dk_nrf5340_cpuapp \
  platforms/nordic/nrf5340/host_hil_endpoint
```
