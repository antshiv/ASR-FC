# Antshiv Flight Controller

Integration repository for Antshiv's deterministic flight-control stack. It
combines reusable numerical libraries through Git submodules, owns embedded
platform adapters, and provides one protocol for SIL, HIL, and direct hardware
links.

```bash
git clone --recurse-submodules \
  https://github.com/antshiv/AntshivFlightController.git
cmake -S . -B build
cmake --build build --parallel
ctest --test-dir build --output-on-failure
```

See [the integration architecture](docs/architecture.md).
