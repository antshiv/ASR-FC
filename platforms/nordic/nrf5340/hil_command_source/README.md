# nRF5340 HIL command source

This test-only Zephyr image emits four versioned ASR-FC motor commands through
the nRF5340 DK UART routed to its J-Link USB serial interface. Every command is
hard-coded disarmed. The differing Q15 values test channel routing but cannot
become PWM or commutation output.

This is not the existing `BLEDroneCode` image and does not contain the CEVA or
sensor drivers. Build it separately before deciding whether to flash it.
