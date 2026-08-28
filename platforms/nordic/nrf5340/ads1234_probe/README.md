# nRF5340 ADS1234 probe

This test-only image verifies the thrust-stand ADS1234 electrical connection
before any motor output is enabled. It reads channel 1 through a two-wire GPIO
interface and prints raw signed 24-bit samples through the nRF5340 DK J-Link
serial port.

The external ADS1234 board must be configured for channel 1, gain 128, and
10 samples per second. `PDWN` connects to Arduino D2/P1.04 so the probe can
perform the required high-low-high startup sequence. The application does not
initialize CEVA, PWM, the KV31F, or the GD3000.
