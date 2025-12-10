# Flashing OSMO Glove Firmware
This guide will review how to flash the firmware for the OSMO glove. We use a custom firmware for the Bosch magnetometers, which enables unique device addresses to support the large quantity of magnetometers in our system. The firmware is completely open-sourced and can be further developed as needed.

## Required Supplies
### Glove Components
- MCU board with sensor boards soldered and connected

### Equipment
- STLinkV3 Programmer
- TagConnect TC2030-IDC-NL
- USB C - USB A cable*

*Note: USB C - USB A cables are strongly recommended over USB C - USB C cables for interfacing with the glove. Due to stricter data transfer standards with USB A, we can only guarantee reliable funcionality with USB C - USB A cables.

### Software
- [STM32 Cube IDE](https://www.st.com/en/development-tools/stm32cubeide.html) (to view/edit firmware)

## Instructions

### 1. Install the STM32 software.
Installation link: [https://www.st.com/en/development-tools/stm32cubeide.html](https://www.st.com/en/development-tools/stm32cubeide.html).

## 2. Plug in and grant sudo permissions to STLinkV3 programmer.
In Terminal:
```
#list USB devices
lsusb

#look for device
Bus 005 Device 006: ID 0483:374f STMicroelectronics STLINK-V3

#grant permission
#specific example
sudo chmod 777 /dev/bus/usb/005/006

#generic example
sudo chmod 777 /dev/bus/usb/<BUS NUMBER>/<DEVICE NUMBER>
```
## 3. Clone the firmware from the osmo_tactile_glove_code repo.
Firmware folder: [Firmware](https://github.com/jessicayin/osmo_tactile_glove_code/tree/main/firmware)

Note that we have two versions: RTOS and non-RTOS. We aim to have the output message match exactly across the two, you can think of RTOS being the multi-threading version and non-RTOS as the single thread version.

RTOS has the advantage of faster output, currently set at 50Hz. Non-RTOS is easier to work with for debugging/print statements and is more stable. We use the non-RTOS version in the paper. 

## 4. Open the firmware
3. Get the firmware from the repo
- RTOS version (50Hz but less stable?)
- stable version (20Hz but more consistent)
4. Load the .ELF files and flash
TODO: generate ELF files for different versions of firmware
Screenshots here

5. Test with screen + port + baud rate 9600
6. Test with basic streaming code (utils/bowie.py)
7. Test with basic visualization (dash_plot_bowie.py)