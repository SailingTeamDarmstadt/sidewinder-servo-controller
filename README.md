# Servo Controller Firmware

This is a servo controller firmware for our custom STM32 Feather board and CAN adapter. It was designed to be a servo/PWM controller for our small testbed Sidewinder/Marlin.

## Features

- Support for different boats via header files / define at the top of main.c
- Receives CAN motor commands from ROS (same addresses as our other boat robooter, making this plug and play with our software stack)
- Does some simple scaling and plausibility checks on received values (so you don't have to calibrate the RC or boat computer)
- Reads PWM from multichannel Remote Control receiver, with an efficient use of timer inputs
- Switches over between RC and CAN control based on RC PWM switch (hard override for the boat computer)
- "Heartbeat" blinking LED
- Reports to USB host as a virtual serial port (115200 8N1), can print verbose debugging messages (toggle via define in source code)

## Programming

- Program using USB DFU mode (preferred) or UART
- Download & install [STM32CubeProgrammer](https://www.st.com/en/development-tools/stm32cubeprog.html)

### USB

- Make sure that Boot0 is set to high (connect the middle pin and the pin closer to the STM of the boot jumper)
  - Reset/power cycle STM32 to (re-)boot in programming mode (or re-plug USB cable)
  - It should show up as something with DFU in the name in `lsusb`.
  - USB port should show up in the CubeProg drop down
  - Click connect, you should now see the chip info in the bottom right corner
- Build firmware in CubeIDE
- Go to the second tab of CubeProg, browser for the firmware .elf file
- "Run after programming" will directly reboot STM in run mode, even if jumper is still set to programming mode

### UART

  - You can use a nucleo programmer in UART mode or any other 3.3V UART adapter 
  - Connect UART adapter to any of the UART ports of the STM
  - Proceed like with USB, select the port in CubeProg 

## Public links

[Public git repository](https://github.com/SailingTeamDarmstadt/sidewinder-servo-controller)

[PCB files](https://github.com/SailingTeamDarmstadt/Featherboard_STM32L452)

## Internal Links:

[Link to PCB files on the Nextcloud](https://nextcloud.sailingteam.hg.tu-darmstadt.de/apps/files/files/100363?dir=/Teams/Sidewinder_Marlin/PCBs) 

[Internal wiki page with more information about the boats](https://gitlab.sailingteam.hg.tu-darmstadt.de/team/wiki/-/wikis/Electronics/Sidewinder/Co-Processor) 

[Internal git repository](https://gitlab.sailingteam.hg.tu-darmstadt.de/e-technik/sidewinder-servo-controller)
