# Troubleshooting

### Glove stops streaming data intermittently
#### Software Issue
Power cycle the glove: completely disconnect the glove from USB-C and reconnect it.

#### Hardware Issue
Please check the solder joints between the sensors and the MCU board, visually checking for frayed wire or checking continuity with a multimeter. Consult your notes on which wires belong to which bus before unsoldering and resoldering. If this is a recurring issue, cover the solder joints directly with UV glue to strengthen the connection mechanically. The UV glue can be peeled off if it needs to be resoldered in the future.

### COBS decode error occurs occasionally or when initializing the glove.
These can be ignored safely.

### IMU axis failed to update (occasionally)
These can be ignored safely. The sensor sends this message when it is static (not moving) for some time, meaning there is no change in signal.

