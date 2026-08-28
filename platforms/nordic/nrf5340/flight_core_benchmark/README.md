# nRF5340 flight-core benchmark

This motor-disabled image measures the portable ASR flight core on the nRF5340
application CPU. A checked `dynamic_models` rigid-body plant starts with a
15-degree roll disturbance. Each 100 Hz iteration converts the simulated state
to a CEVA-shaped attitude frame, executes the checked quaternion PID and
geometry mixer, advances the plant with RK4, and feeds the result back into the
next iteration. It contains no PWM, ESC, or motor-link driver.

The benchmark is an execution-capacity gate, not flight evidence. Live CEVA
SPI acquisition is validated separately before the two paths are joined.

```bash
west build -b nrf5340dk/nrf5340/cpuapp \
  platforms/nordic/nrf5340/flight_core_benchmark
west flash
```
