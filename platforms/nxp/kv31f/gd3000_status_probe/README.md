# KV31F GD3000 read-only status probe

This image validates communication with the GD3000 without enabling a motor.
It sends only the read-only `NULL0` through `NULL3` commands and prints the
returned status bytes through the FRDM-KV31F OpenSDA UART. It follows NXP's
AN5169/MC33927 implementation: each requested register command is followed by
a `NULL0` transfer, and the response captured during that second transaction
is the requested status byte.

## Safety boundary

- All six PWM pins are configured as GPIO outputs held low.
- `EN` is held low for the complete lifetime of the image.
- `RST_B` is released only after PWM, enable and serial pins are initialized.
- No FTM, PWM, ADC or DSPI peripheral is initialized.
- The image exposes no register-write, fault-clear, calibration or gate-enable
  path.
- Keep motor and inverter power disconnected while running this probe.

The six gate-input pins are initialized low while `EN` remains low. This is
not an enable-ready PWM state: GD3000 high-side inputs are active-low. Gate
enable requires a separately reviewed FTM polarity, jumper, dead-time, and
fault-handling implementation.

The GD3000 ANDs its enable inputs, so the low `EN` line keeps its gate outputs
disabled even after `RST_B` is released for status communication.

## FRDM-GD3000 connector mapping

| Signal | KV31F pin |
| --- | --- |
| `RST_B` | `PTA1` |
| `INT` | `PTE1` |
| `EN` | `PTC6` |
| `CS` | `PTC19` |
| `MOSI` | `PTC18` |
| `MISO` | `PTC17` |
| `CLK` | `PTC16` |
| `PWM2`, `PWM1`, `PWM0` | `PTC1`, `PTC2`, `PTC5` |
| `PWM3`, `PWM4`, `PWM5` | `PTC4`, `PTD4`, `PTD5` |

The mapping follows Table 9 of NXP's FRDM-GD3000EVB Rev. 3 user guide for the
physical J2A connection to an FRDM-KV31F. The recovered
`TPP_FRDM-KV31F_BldcControl` sample instead muxes `PTD6` and `PTE17` through
`PTE19`; those pins do not map to the shield's J2A SPI signals in that table.
The probe therefore uses slow GPIO mode-1 transfers on the documented physical
connector pins without enabling the DSPI, PWM, or motor-control paths.

## Build

```bash
export SdkRootDirPath=/data/Workspace/NXP/mcux-2.12-frdmkv31f
cmake -S armgcc -B armgcc/build \
  -DCMAKE_TOOLCHAIN_FILE="$SdkRootDirPath/core/tools/cmake_toolchain_files/armgcc.cmake" \
  -DCMAKE_BUILD_TYPE=Debug
cmake --build armgcc/build --parallel
```

Expected reset status is commonly `0x80`. A constant `0x00` or `0xFF` across
all four registers should be treated as wiring, power, reset or serial-timing
evidence, not as a passing GD3000 status.
