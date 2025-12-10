# BowieGlove Firmware

We have two versions - `rtos` and `non-rtos`. We aim to have the output message match exactly across the two, you can think of `rtos` being the multi-threading version and `non-rtos` as the single thread version.

`rtos` has the advantage of faster output, currently set at 50Hz.
`non-rtos` is easier to work with for debugging/print statements.

## Building

Note: Create two separate workspaces, the projects will override each other.
Open your STM32CubeIDE workspace and select File --> Import Projects from existing File System. Point the directory location to `../hardware/glove/fw/BowieGlove` or `../hardware/glove/fw/BowieGlove_rtos` to open the project.

Open the main.c under `Core/src/main.c`. Select the hammer icon to build this project. Click the green arrow to build and flash the binary to the glove MCU. (requries ST V3set)

Can use PuTTY to view the binary data streaming over USB. Decode using `BowieGlove` class 
