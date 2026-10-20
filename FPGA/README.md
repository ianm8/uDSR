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

# Summary

The worst point is 173.0 kHz above the NCO. At the CIC's decimation to 187.5 kHz it folds by one sample rate, to 173.0 − 187.5 = **−14.5 kHz**. That's inside the final ±15.6 kHz output, so the FIR's ÷6 stage doesn't reduce it. It arrives near the left-hand edge of the display, about 0.4 dB into the FIR's roll-off, at −65 dB (not noticable).

That's the general pattern for CIC aliases: the attenuation depends on where they land.

| Lands at (output) | Comes from | CIC attenuation |
|---|---|---|
| ±14.5 kHz (display edge) | 187.5 ∓ 14.5 kHz | −65 dB |
| ±3 kHz (edge of an SSB channel) | 187.5 ∓ 3 kHz | −107 dB |
| ±1 kHz (middle of the audio) | 187.5 ∓ 1 kHz | −136 dB |
| 0 Hz (DC) | 187.5 kHz exactly | null (in theory unlimited) |

The CIC's nulls fall exactly on multiples of 187.5 kHz, so anything that would fold onto DC, or near it, sits in a deep null. The alias protection is strongest where you actually listen, in the middle of the passband, and weakest at the display edges.

The FIR's ÷6 stage aliases are much flatter by comparison. Anything that folds into the ±3 kHz audio is at least 74 dB down, and the worst case anywhere is −73.5 dB. So for the audio channel, the limit is the FIR at about −74 dB, not the CIC. The −65 dB figure only matters for a strong signal 173 kHz away, which would show as a faint trace at the edge of the spectrum display.
