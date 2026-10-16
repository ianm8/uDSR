# Firmware objects

Use these files to program the FPGA and the RP2350. No need to install the GOWIN IDE or the Arduino IDE.

## FPGA File Stream
Use openFPGALoader to program the FPGA

* start MSYS2
* openFPGALoader --detect
* openFPGALoader -b tangnano9k -f uDST.fs

If successful, the scrolling pattern should appear on the 6 LEDs.

## RP2350 UF2

* Put the RP2350 into bootloader mode (press the *Boot* button while plugging in the USB cable, or press the reset button).
* Copy the UF2 file into the directory that pops up.

If successful, the LED should flash the error code for FPGA missing.
