# LS012B7DD06 on ESP32 example
Example user program showing the use of my LS012B7DD06 driver for ESP32. The code needs a clean-up, but provides a working example. This description will be updated in near future.

This project was developed on ESP32 PICO KIT 1 incorporating the ESP32-PICO-V3 chip. A brief description of the driver is at the time in main/rlcd_lib/ls012b7dd06.h. Timing values and pin connections are defined in main/rlcd_lib/ls012b7dd06_hal.h. Useful links to projects this one was based on are in main/rlcd_lib/i2s_parallel_driver/i2s_parallel.h.

In order for this code to work, additional external hardware is needed between the ESP32 and the LCD:
- 1 NOT gate (e.g. 74LVC1G14)
- 2 D flip-flops (e.g. 74HC74)
- 2 switching diodes
- 3-4 pull-up and pull-down resistors
The external hardware shown on schematic_external_hardware.pdf is used to block constant bit clock wave (BCK) between rising edges of GEN and BSP, that is force BCK low during this period.

Regarding the following logic values, the author is not sure whether they are correct.
ST1 and ST2 labels, standing for state of the flip-flop #1 and state of the flip-flop #2, on the schematic can be connected to ESP32 digital input pins to detect initial states of the flip-flops. The flip-flops must be preloaded with the following values before initializing the display:
- ST1 must be initially low
- ST2 must be initially high

Before init of I2S:
- if ST1 is low at startup, send a single pulse on I2S_CLK line.
- if ST2 is high at startup, send a single pulse on I2S_GEN or I2S_BSP.