# About the uDSR

The uDSR is a small software defined direct sampling HF receiver. It uses the ADC10080 from TI clocked at 81 MHz. The FPGA is from Brisbane Silicon which is based on the ??? from GOWIN. The receiver features a single band-pass filter, MMIC and 16:1 transformer (12 dB voltage gain). After building and using the uDST, the following questions arose that I wanted answers to:

 * uDST has good large signal handling so can we get away with a single bandpass filter?
 * I suggested swapping out the 27 MHz oscillator in the BR100 for a 24 MHz TCXO giving a sample rate of 80 MHz. It works well, but how well does the standard 27 MHz oscillator work with a sample rate of 81 MHz?
 * In the uDST (and the MBPTRX) the audio amp has a dedicated 5V regulator since without it there was interference to the spectrum display on the MBPTRX at higher volumes. What happens when this is removed?
 * uDST uses a (expensive) 12 bit ADC. How well does the receiver work with a lower cost, 10 bit ADC?
 * I needed to save a pin in order to use the Xiao RP2350 (instead of a Pico 2) so I'm using the ADC to process the rotary encoder. Does it work?

# FPGA

## FPGA function

The FPGA is a Brisbane Silicon **BRS-100-GW1NR9** board (Gowin **GW1NR-LV9**). It runs a dedicated digital down-converter, **DDC26**. The DDC turns the 81 MSPS stream from the ADC into 16-bit I/Q samples at 31.25 kHz for the RP2350.

- **Clock:** the on-chip rPLL multiplies the board's 27 MHz oscillator by 3. This gives the 81 MHz system clock and ADC sample clock.
- **Mixer:** a 32-bit NCO, set by a tuning word from the RP2350, generates sine and cosine. These multiply the ADC samples, shifting the wanted frequency to 0 Hz as complex I/Q. Tuning resolution is about 0.02 Hz.
- **CIC decimator:** a 3-stage CIC filter decimates by 432, from 81 MHz to 187.5 kHz.
- **FIR decimator:** a 252-tap FIR filter decimates by 6, from 187.5 kHz to 31.25 kHz. It provides the final anti-alias shaping, flat to about 14 kHz with about 67 dB stopband, and corrects the CIC droop.
- **Scaling:** each stage rounds and saturates, and the overall gain is ×32. A full-scale ADC sine therefore maps to 16-bit full scale, so digital clipping coincides with ADC clipping. Rounding to 16 bits costs only about 1 dB of sensitivity.
- **Interface:** every 32 µs the FPGA raises a data-ready strobe. The RP2350 then exchanges one 32-bit SPI word with it: I and Q come out, and the next NCO tuning word goes in, in the same transfer.

---

## Circuit

### RF front end

- **Protection:** 1N914 clamp diodes with 10 nF capacitive coupling limit large input voltages to roughly ±0.7 V. This protects the MMIC from strong nearby signals and static.
- **Band-pass filter:** a 5th-order high-pass (3 MHz) is cascaded with a 9th-order low-pass (32 MHz).
  - The low-pass is the anti-alias filter. It keeps signals inside the first Nyquist zone (0–40.5 MHz) so out-of-band signals can't fold onto HF.
  - A 1 kΩ resistor across the high-pass gives a flat ~25 dB attenuation below 3 MHz. The AM broadcast band and LF can still be received, but strong stations there can't overload the ADC.
- **Amplifier:** an **AG203-63G MMIC** provides +20 dB of gain with a 3.1 dB noise figure.
- **Transformer:** a **16:1 transformer** adds a further +12 dB of voltage gain and converts single-ended to differential. Its primary is terminated in 50 Ω, and it drives the high-impedance ADC inputs directly.

This extra analog gain lifts the receiver's noise well above the 10-bit ADC's own noise floor, recovering the sensitivity the lower-resolution ADC would otherwise lose. Estimated performance:

| Parameter | Value |
|---|---|
| Net gain, antenna → ADC | ≈ +30.5 dB |
| System noise figure | ≈ 12 dB |
| ADC full scale at antenna | ≈ −26.5 dBm (S9+46) |
| MDS, 2.4 kHz SSB | ≈ −128 dBm |

*Estimates assume ~1 dB filter loss and ~0.5 dB transformer loss.*

### ADC

- **Part:** TI **ADC10080**, 10-bit, 1 Vpp differential input, 60 dB SNR.
- **Clock:** clocked at 81 MSPS from the FPGA. That's slightly above the part's 80 MSPS rating.

### Microcontroller

The **RP2350** sits on a Seeed XIAO board and runs at 250 MHz.

- **Core 1** runs the DSP: SSB, CW and AM demodulation, AGC, noise blanker and notch.
- **Core 0** runs the user interface, the 1024-point spectrum display, and a 128×32 OLED over I²C.

The FPGA link is a PIO-based SPI at 6 MHz, plus the data-ready input.

### Audio output

Two 6-bit PWM outputs at 3.9 MHz (250 MHz ÷ 64) form a 12-bit DAC.

- **Summing:** the high-order PWM feeds the summing node through 1.5 kΩ and the low-order PWM through 100 kΩ. That's a weighting of about 67:1, against the ideal 64:1.
- **Filtering:** 100 nF and 470 Ω to ground at the node make a first-order low-pass at about 4.5 kHz. This attenuates the 3.9 MHz PWM carrier by about 59 dB and scales the output to about 0.8 Vpp.
- **Amplifier:** a 100 nF series capacitor AC-couples the audio into a TI **LM4875** amplifier, which has DC voltage-controlled volume, driving headphones.

### Tuning control

- **Encoder:** the rotary encoder's A and B contacts, with the common at 3.3 V, go through an R / 2R / R resistor network. This produces four voltage levels on a single RP2350 ADC pin, saving a GPIO on the 11-pin XIAO.
- **Decoding:** firmware decodes the levels with a settling filter that rejects the intermediate voltages seen as the contacts change.
- **Button:** the encoder push button uses a digital pin.

### Power

The board runs from **13.8 V**. A 10 Ω 2 W series resistor feeds an **MC7805** linear regulator. The resistor takes part of the voltage drop, sharing the heat that the regulator would otherwise dissipate alone.
