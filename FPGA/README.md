# FPGA Programming

## Project

**If you just want to program the FPGA, skip to the Device Programming section below.**

In GOWIN, just create a new project with device GW1NR-LV9QN88PC6/I5, add these files, and build.

**Note: need to set dual function pin as follows:**
* goto Project, Configuration
* Place & Route, Dual-Purpose Pin
* Check: Use SSPI as regular IO

## Device Programming
* start MSYS2
* openFPGALoader --detect
* openFPGALoader -b tangnano9k -f uDSR.fs

# FPGA Block Diagram

![uDST FPGA](https://github.com/ianm8/uDSR/blob/main/docs/uDSR_FPGA_DDC26.png?raw=true)


# Frequency Response

![uDST FPGA](https://github.com/ianm8/uDSR/blob/main/docs/uDSR_DDC_combined_response.png?raw=true)

