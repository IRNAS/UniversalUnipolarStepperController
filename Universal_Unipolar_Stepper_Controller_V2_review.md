# Universal Unipolar Stepper Controller

Revision of hardware and software for Universal Unipolar Stepper Controller. The main guidelines when considering new design:
- change to ARM microcontroller
- port new software to microcontroller - GRBL
- better power supply for microcontroller and laser, change linear to switching
- measurement of consumption 
- unipolar and bipolar stepper drivers (unipolar is must have)
- board dimension smaller then 50x50 mm
- all current connectors should be on the new board
- native USB on microcontroller and USB connector on board
- developed with open source software
- Arduino or something similar compatible

#### Typical use-cases
- [KORUZA project](http://koruza.net/)
- open-source microscope automation system
- optics lab micrometer drive motorization
- mini 3D printers and robots
- CNC machines, etc.


## 1. Microcontroller (ARM)
Currently there is MSP430G2955 controler on the stepper driver board. This controler costs 3.19 USD, and we want to put cheaper one. Because of the main guidelines from above and new firmware (this will be discussed in Firmware section), microcontroller requirements are:
-  ROM (FLASH) memory must be bigger than 32 KB
-  must have EPROM memory
- at least two ADC for power consumption measurement
- enough GPIOs (~25) for driving stepper drivers, encoders, limit switches 
- communication, etc. 
- UART, SPI, I2C communication
- native USB 
- open source programing environment, Arduino if possible
- unit price smaller then 3 USD

Because of the new firmware, and some additional features like USB and encoders it is good to migrate to new cheaper ARM platform.

## 2. Firmware (GRBL)

Firmware for new revision of Universal Unipolar Stepper Controller will be based on GRBL software. GRBL is software for controlling the motion of machines like CNCs, 3D printers, laser cutters, etc. GRBL is based on Arduino Uno (ATmega328P), but the newer version of software also supports Arduino Mega (ATmega2560). Source code is written in highly optimized C for the ATmega328P microcontroller. They have exhausted almost all its capabilities, and filled the memory (Firmware size is 32 KB). So if we want some new features, new platform is the best solution (ARM). There are projects where people port GRBL firmware to ARM architecture processors, so it is possible. Currently GRBL development team is making new 3D printer and they are writing firmware for ARM, but for now that is all we know. 

GRBL firmware does not support H bridge drivers for driving stepper motors (ULN2003, L295, etc.).

GRBL is good solution for new firmware because it is widely used, professionally written, have good documentation and open source. It should not be a problem to port it to some other architecture and add some new features.

## 3. DC power supply

Current version of the Universal Unipolar Stepper Controller have linear power supply for powering microcontroller and laser (used in KORUZA project). Problem is big heat dissipation because regulator maximum current limit is 800 mA and laser requires about 500 mA to work properly. Additional requirement is that the input voltage should be 24 V, or better 30 V if possible. Output voltage is 3.3 V

Considering all, it is best that new power supply is switching based, because of higher output current, and higher input voltage. That will prevent big heat dissipation.

## 4. Measurement of consumption
All power consumed by the controller must be measured/reported by itself, not accurately, but say in the 20-50 mA accuracy range. There are several ways to measure current consumption, with shunt resistor and op-amp, shunt resistor and two ADCs, special chips working on Hall effect etc.

Because current measurement does not have to be precise, and we already have ADCs in microcontroller, the best and cheapest solution is to put shunt resistor 0.1 Ohm and measure voltages on his ends with two ADCs built in microcontroller. 

## 5. Stepper motor drivers
There are three unipolar stepper motor connectors on current version of the board. New revision of the board must support unipolar stepper motors, and if possible it should support bipolar stepper motors too. Because of the GRBL firmware which does not support drive of H bridges, drivers must be changed. 

Solutions for this are dedicated stepper motor drivers, for unipolar stepper motors. They have two pin interface (direction and step pulse pin), so this will reduce number of pins needed on microcontroller. This drivers also have overcurrent protection and thermal shutdown protection

## 6. PCB and components
Current board dimensions are 45 x 45 mm, and new board should be the same or not bigger than 50 x 50 mm. On the board are connectors:
- 2x power supply
- 1x Universal Serial Connector (UART, SPI, I2C)
- 3x unipolar stepper motor connectors
- 3x encored connectors
- 1x programming/debugging connector
- 1x end switch header connector
- 1x laser connector
and two push buttons on the side. All of this connectors must be on the new version of the board with addition of USB connector. Component package can not be smaller than 0603.
Design rules can be found on this web link: [DESIGN RULES](http://dirtypcbs.com/about.php) 







