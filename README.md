# Nori Clock

This repo contains the firmware for the Nori Clock, the revival of the now decommissioned Phone Call Monitor (a.k.a.
Phone Call Monitoring Facility) device.

## Build

The firmware can be built in **MPLAB IDE v8.92**, using the provided MCP-file.

### Prerequisites

* The MPLAB **C18** compiler must be installed. (This project is using version **3.47**.)
* The location of the `18f4520_g.lkr` file may have to be updated based on your environment.

## Version History

* 1.0.0 - Initial commit
* 1.1.0 - Add brightness control upon regular button press
* 1.2.0 - Add 12/24-hour mode
* 1.3.0 - Add support for 3 date formats
* 2.0.0 - Switch to 16 MHz TCXO external oscillator
* 2.1.0 - Add DST auto-adjustment option
* 2.2.0 - Reload Timer 0 before compiler-managed ISR context save
* 2.3.0 - Add more display modes
* 2.3.1 - Disable integer promotions
* 2.3.2 - Calculate initial DOW instead of hardcoding

## Author Info

Email: apc067@gmail.com

Project link: https://www.nixiana.com/projects/nori_clock/
