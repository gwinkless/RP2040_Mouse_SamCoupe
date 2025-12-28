# Copyright
2025 Geoff Winkless (github@geoff.dj)

Based on the work by:

- 2023 Darren Jones (nz.darren.jones@gmail.com)
- Simon Inns https://github.com/simoninns/SmallyMouse2
- sekigon-gonnoc https://github.com/sekigon-gonnoc/Pico-PIO-USB

# RP2040 Mouse
This is the software component of my Sam Coupé version of Darren's RP2040 based USB to Quadrature mouse adaptor.

Hardware will be available from PCBWay once I've finished designing it, or you can build your own version with a Pico, a DIN8 (262°) connector, two 74HC03 open-drain NAND gates, a couple of 100nF capacitors, a 300Ohm pullup resistor and a USB OTG adapter.


#### Why did you make it?

The Sam Coupé uses a unique(?) 5-pin bus mouse connection with deltas transmitted on-demand.

The existing interfaces for the Sam require either a PS2 (or PS2-over-USB) mouse or a USB-HID mouse with something like the rp2040-mouse (or Smally mouse) and then the SamCo mouse interface to convert the Quadrature output into Sam-specific signals. That's expensive and clunky, and the SamCo interfaces are rare.

# Building

To build this software you will need:
- a working TinyUSB SDK (https://github.com/hathach/tinyusb)
- CMake Tool Chain

To build the software clone this repository and then run the following
```
PICO_TINYUSB_PATH=/path/to/tinyusb BOARD=pio_sdk cmake ..
make
```

Once the build completes you will see the following:

```
Memory region         Used Size  Region Size  %age Used
           FLASH:       37536 B         2 MB      1.79%
             RAM:        8220 B       256 KB      3.14%
       SCRATCH_X:          2 KB         4 KB     50.00%
       SCRATCH_Y:          2 KB         4 KB     50.00%
```

# Installing

Press and hold the Pico USB-boot button, connect to your computer using the Pico USB, then copy `rp2040_mouse.uf2` built above to the USB drive that the board will expose.
