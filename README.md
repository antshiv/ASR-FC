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

See [the integration architecture](docs/architecture.md) and the
[first disarmed hardware bring-up](docs/hil-bringup-2026-08-24.md). The
[nRF5340 closed-loop SIL gate](docs/nrf5340-flight-loop-2026-08-26.md) records
the first measured controller, mixer, and rigid-body loop on the physical MCU.
