# ASR-FC: Antshiv Robotics Flight Controller

**ASR-FC** stands for **Antshiv Robotics Flight Controller**. This is the
integration repository for Antshiv's deterministic flight-control stack. It
combines reusable numerical libraries through Git submodules, owns embedded
platform adapters, and provides one protocol for SIL, HIL, and direct hardware
links.

```bash
git clone --recurse-submodules \
  https://github.com/antshiv/ASR-FC.git
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

Run the same sanitizer-enabled harness used by pull requests and nightly CI:

```bash
scripts/run_validation.sh
```

The harness writes compiler, submodule, runner-hardware, build-log, and JUnit
evidence under `artifacts/validation/`.

The motor-disabled [host HIL endpoint](platforms/nordic/nrf5340/host_hil_endpoint/README.md)
accepts versioned synthetic sensor and guidance frames, runs the same checked
flight core on the nRF5340, and returns virtual motor commands for a host plant.
Its protocol and endpoint state machine run in native pull-request and nightly
tests before the Zephyr image is built or flashed.

See [the integration architecture](docs/architecture.md) and the
[first disarmed hardware bring-up](docs/hil-bringup-2026-08-24.md). The
[nRF5340 closed-loop SIL gate](docs/nrf5340-flight-loop-2026-08-26.md) records
the first measured controller, mixer, and rigid-body loop on the physical MCU.
The [host HIL contract](docs/host-hil-contract-2026-08-27.md) defines the next
lockstep AeroDyn integration boundary.
