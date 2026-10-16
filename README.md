# uDSR
A direct sampling SDR receiver, largely based on the receiver section of the uDST.

# uDSR vs uDST

uDSR comes out about 1.8 dB more sensitive than uDST, but it gives up 12 dB of clipping headroom. The 10-bit ADC's 10 dB worse SNR is more than paid back by the halved full-scale voltage and the 6 dB of extra transformer gain.

**Assumptions:**
- Antenna-referred, 50 Ω.
- 1 dB filter loss, then the AG203-63G (20 dB gain, 3.1 dB NF), then 0.5 dB transformer loss.
- The transformers give pure voltage step-up into the ~100 kΩ ADC inputs, with the MMIC loaded by the 50 Ω primary resistor.
- The SNR figures are dBFS over Nyquist.

| | uDST | uDSR |
|---|---|---|
| Net voltage gain, antenna → ADC | 24.5 dB | 30.5 dB |
| ADC full scale (50 Ω-equivalent) | +10.0 dBm | +4.0 dBm |
| **Full scale at antenna** | **−14.5 dBm (S9+58.5)** | **−26.5 dBm (S9+46.5)** |
| ADC noise density | −146.0 dBFS/Hz | −136.1 dBFS/Hz |
| ADC noise at ADC input | −136.0 dBm/Hz | −132.1 dBm/Hz |
| Analog chain noise at ADC input | −145.4 dBm/Hz | −139.4 dBm/Hz |
| Dominant noise source | ADC, by 9.4 dB | ADC, by 7.3 dB |
| **System noise figure** | **14.0 dB** | **12.1 dB** |
| Noise floor at antenna | −160.0 dBm/Hz | −161.9 dBm/Hz |

**MDS (noise power in bandwidth, antenna-referred)**

| Bandwidth | uDST | uDSR |
|---|---|---|
| 1 Hz | −160.0 dBm | −161.9 dBm |
| 30.5 Hz (display bin, before window ENBW) | −145.2 dBm | −147.0 dBm |
| 500 Hz CW | −133.1 dBm | −134.9 dBm |
| 2.4 kHz SSB | −126.2 dBm (0.11 µV) | −128.1 dBm (0.088 µV) |
| 2.4 kHz SSB at 10 dB SNR | 0.34 µV | 0.28 µV |

**Dynamic range in channel (full-scale sine to noise floor)**

| Bandwidth | uDST | uDSR |
|---|---|---|
| 1 Hz | 145.5 dB | 135.4 dB |
| 500 Hz | 118.6 dB | 108.4 dB |
| 2.4 kHz | 111.7 dB | 101.6 dB |

**Notes:**

- **Both radios are ADC-limited**, so extra analog gain still improves the NF, but every dB of gain costs a dB of headroom. If uDSR used uDST's 4:1 transformer instead of the 16:1, it would come out at about 17.6 dB NF with −20.5 dBm full scale. The 16:1 choice buys 5.5 dB of NF for 6 dB of headroom, which is a good trade.
- **In practice, below about 14 MHz band noise swamps both radios.** At 10 m, galactic noise (Fa around 18–19 dB) sits about 5 dB above uDST's receiver noise and about 7 dB above uDSR's. Receiver noise then adds roughly 1.2 dB and 0.8 dB respectively to what you hear, before any antenna or feedline loss.
- **Not included:**
  - ADC SFDR and spurs (you have 88 dBc for the AD9236; I don't have a figure for the 10-bit part).
  - MMIC IP3 limits.
  - Digital-chain quantisation.
- **On quantisation:** the uDSR noise floor sits about 10 dB higher relative to ADC full scale than uDST's. So whatever CIC output shift you choose, check that it puts the noise floor several LSBs above 16-bit quantisation, as uDST's >>22 does. If it does, the output word won't degrade any of the figures above.
