# KV31F platform

This platform will implement the generic `motorDynamics` HAL using the official
MCUXpresso KV31F BSP, FTM/PWM, ADC/PDB, DSPI, and GD3000 fault/status handling.
All bridge outputs must initialize inactive. No enabled output is accepted until
the board identity, timing, pre-driver communication, and fault gates pass.
