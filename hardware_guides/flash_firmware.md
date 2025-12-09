# Flashing OSMO Glove Firmware

Required:

Electronics
- STM32 flasher
- pin
- USB C to USB A cable

Software
- STM32 IDE (if want to view/edit code)
- STM32 programmer (if just want to flash software)

1. Install the software (link)
2. Ubuntu + MacOS permissions
(include screenshots as necessary)
3. Get the firmware from the repo
- RTOS version (50Hz but less stable?)
- stable version (20Hz but more consistent)
4. Load the .ELF files and flash
TODO: generate ELF files for different versions of firmware
Screenshots here

5. Test with screen + port + baud rate 9600
6. Test with basic streaming code (utils/bowie.py)
7. Test with basic visualization (dash_plot_bowie.py)