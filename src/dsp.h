#ifndef DSP_H
#define DSP_H

#include "filter.h"
#include "notch.h"
#include "util.h"

namespace DSP
{
  static notch_t notch;
  volatile static float agc_peak = 0.0f;
  volatile static float meter_level = 0.0f;

  static const int32_t __not_in_flash_func(get_agc)(void)
  {
    return (int32_t)agc_peak;
  }

  static const int16_t __not_in_flash_func(agc)(const float in)
  {
    // limit gain to max of 200 (46db)
    //// testing 50 (34 dB)
    static const float max_gain = 50.0f;
    // decay about 10dB per second
    static const float k = 0.99996f;

    const float magnitude = fabsf(in);

    // average detector for the meter: mean|x| x pi/2 reads a steady carrier
    // the same as agc_peak, but reads noise ~9 dB lower (no peak-holding)
    static constexpr float meter_alpha = 0.0005f;   // ~64 ms at 31.25 kHz
    meter_level += (magnitude * 1.5708f - meter_level) * meter_alpha;

    if (magnitude>agc_peak)
    {
      agc_peak = magnitude;
    }
    else
    {
      agc_peak *= k;
    }

    // trap issues with low values
    if (agc_peak<1.0f) return (int16_t)(in * max_gain);

    // set maximum gain possible for 12 bit DAC
    const float m = 2047.0f/agc_peak;
    return (int16_t)(in*fminf(m,max_gain));
  }

  // -------------------------------------------------------------------------
  // S-meter
  //
  // Two steps:
  //   1. measured level -> dBm at the antenna, from ONE calibration constant
  //   2. dBm -> bar pixels, from a table (SMETER_SCALE) shaped like the meter
  //      you want to match, here the FTDX10 with AMP1 off (IPO)
  //
  // Step 1: AGC_AT_MINUS73 is the detector reading for a -73 dBm carrier,
  // measured with the generator (155 put a -73 dBm carrier on the '9').
  //
  // Step 2: commercial S-meters are not 6 dB per S-unit all the way down.
  // The low end is compressed (about 4 dB per unit) so band noise reads low,
  // and the FTDX10's S9 is calibrated with AMP1 on, so in IPO a true -73 dBm
  // shows as about S7. The default table reproduces both:
  //
  //   dBm   -105  -101   -93   -85   -73   -61   -41   -21
  //   reads   S0    S1    S3    S5    S7    S9  +20   +40
  //           <- 4 dB/unit -><- 6 dB/unit ->
  //
  // To fit it to your DX10: set the generator to a few levels (e.g. -61,
  // -73, -85, -93, -101 dBm), note what the DX10 reads in IPO, and put
  // those pairs in the table. Bar pixels for each scale mark on
  // "1  3  5  7  9 +20 +40":
  //   S1=2  S2=11  S3=20  S4=29  S5=38  S6=47  S7=56  S8=65  S9=74
  //   S9+10=86  S9+20=98  S9+30=110  S9+40=122
  // Between knots the bar moves linearly in dB. Knots must be in rising dBm.
  // -------------------------------------------------------------------------
  //#define SMETER_AVERAGE

  volatile static float meter_dbm = -200.0f;   // last S-meter level, dBm at the antenna
  //static constexpr float AGC_AT_MINUS73 = 155.0f;   // detector reading at -73 dBm
  // detector reading for a -73 dBm carrier, per mode (measured with the generator)
  static constexpr float AGC_AT_MINUS73_SSB = 310.0f;  // phasing sum doubles a tone
  static constexpr float AGC_AT_MINUS73_CW  = 140.0f;  // 700 Hz BPF has ~half the gain
  static constexpr float AGC_AT_MINUS73_AM  = 275.0f;  // measured with the generator
  volatile static float meter_ref = AGC_AT_MINUS73_SSB; // set by the active demodulator

  struct smeter_knot_t { float dbm; float px; };
  static const smeter_knot_t SMETER_SCALE[] =
  {
    {  -97.0f,   0.0f },   // below this: no bar
    {  -95.0f,   2.0f },   // S1   (40 m CW noise)
    {  -85.0f,  20.0f },   // S3   (40 m SSB noise: DX10 S3)
    {  -81.0f,  38.0f },   // S5
    {  -77.0f,  56.0f },   // S7
    {  -73.0f,  74.0f },   // S9   (true S9)
    {  -53.0f,  98.0f },   // S9+20
    {  -33.0f, 122.0f },   // S9+40
    {  -27.0f, 127.0f }   // end of the bar
  };
  static constexpr uint32_t SMETER_KNOTS = sizeof(SMETER_SCALE)/sizeof(SMETER_SCALE[0]);

  static const uint8_t __not_in_flash_func(smeter)(void)
  {
#ifdef SMETER_AVERAGE
    const float level = meter_level;
#else
    const float level = agc_peak;
#endif
    if (level < 1.0e-3f)
    {
      return 0u;
    }
    // step 1: level -> dBm at the antenna
    const float dbm = -73.0f + 20.0f * log10f(level / meter_ref);
    
    // debug value
    meter_dbm = dbm;

    // step 2: dBm -> pixels, linear between the table's knots
    if (dbm <= SMETER_SCALE[0].dbm)
    {
      return 0u;
    }
    for (uint32_t i=1;i<SMETER_KNOTS;i++)
    {
      if (dbm < SMETER_SCALE[i].dbm)
      {
        const smeter_knot_t &a = SMETER_SCALE[i-1];
        const smeter_knot_t &b = SMETER_SCALE[i];
        const float px = a.px + (dbm - a.dbm) * (b.px - a.px) / (b.dbm - a.dbm);
        return (uint8_t)(px + 0.5f);
      }
    }
    return 127u;
  }

  static void __not_in_flash_func(noise_blanker)(float &I, float &Q, const uint8_t level)
  {
    // Impulse ("pop") blanker working on the 31.25 kHz I/Q, before any
    // narrow filtering.
    //
    // Small events are blanked exactly as the original blanker did:
    // GUARD_PRE samples before + the samples over the threshold. Most
    // triggers at levels 3-5 are noise peaks and voice transients, so this
    // keeps the damage to signals the same as before.
    //
    // Big pops get more. By the time a fast pop reaches here, the DDC's
    // 252-tap linear-phase FIR has spread it into a burst that rings on BOTH
    // sides of its peak, falling only ~2.2 dB per sample:
    //
    //   offset (samples)  0    +/-1   +/-2   +/-4   +/-6   +/-10   +/-14
    //   level   (dB)      0    -2.9  -11.6  -19.2  -24.1  -32.6   -37
    //
    // Blanking only the samples over the threshold leaves that ringing, which
    // still clicks. So a pop more than BIG_POP_DB over the threshold also
    // gets 'extra' samples blanked on each side, one per 2.2 dB it rose above
    // that, enough for the ringing to decay back to the threshold:
    //
    //   GUARD_PRE + extra  |  pop  |  extra
    //
    // States:
    //   NB_IDLE    watching; average tracks the signal
    //   NB_EVENT   amplitude above threshold, peak recorded
    //   NB_POST    big pop over; waiting 'extra' samples for its trailing
    //              ringing (a new pop inside this window is merged)
    //   NB_SIGNAL  ran longer than MAX_BLANK_RUN: a real signal (voice peak,
    //              nearby station), passed untouched while the average adapts
    //
    // Delay: NB_LEN samples (64 / 31250 = ~2 ms) while enabled, none when off.

    static constexpr uint32_t NB_LEN = 64u;          // delay line, power of two
    static constexpr uint32_t NB_MASK = NB_LEN - 1u;
    static constexpr uint32_t MAX_BLANK_RUN = 32u;   // ~1 ms: longer = signal, not a pop
    static constexpr uint32_t GUARD_PRE = 2u;        // always blanked before an event (as before)
    static constexpr float BIG_POP_DB = 6.0f;        // pops this far over the threshold get extra guard
    static constexpr uint32_t GUARD_EXTRA_MAX = 12u; // extra samples each side, biggest pops (~380 us)
    static constexpr float RING_DB_PER_SAMPLE = 2.2f;// DDC FIR ringing decay (measured)
    static constexpr float alpha = 0.002f;           // average, ~16 ms at 31250 Hz
    static_assert(GUARD_PRE + 2u*GUARD_EXTRA_MAX + MAX_BLANK_RUN <= NB_LEN - 2u, "blank window must fit in the delay line");

    // thresholds, number of times greater than
    // average to trigger a blanking event
    static const float thresholds[] =
    {
      0.0f, // level 0 — unused (returns early)
      9.0f, // level 1 — only very large spikes
      7.0f, // level 2 — conservative
      5.0f, // level 3 — moderate
      3.0f, // level 4 — fairly aggressive
      2.0f  // level 5 — maximum blanking
    };

    enum nb_state_t { NB_IDLE, NB_EVENT, NB_POST, NB_SIGNAL };
    static struct
    {
      float bufI[NB_LEN];
      float bufQ[NB_LEN];
      float avg_amp;
      float peak;       // largest amplitude in the current event
      uint32_t w;       // delay line write index
      uint32_t run;     // samples in the event
      uint32_t post;    // samples since the event ended
      uint32_t extra;   // extra guard for this event, each side (0 for small pops)
      nb_state_t state;
      uint32_t active;
    } s = {};

    if ((level == 0u) || (level > 5u))
    {
      s.active = 0u;     // bypass; state and delay line refill on re-enable
      return;
    }
    const float threshold = thresholds[level];

    if (s.active == 0u)
    {
      for (uint32_t i = 0u; i < NB_LEN; i++)
      {
        s.bufI[i] = I;
        s.bufQ[i] = Q;
      }
      s.avg_amp = 0.01f;
      s.w = 0u;
      s.run = 0u;
      s.post = 0u;
      s.state = NB_IDLE;
      s.active = 1u;
    }

    // wideband instantaneous amplitude (fast L1 estimate)
    const float amp = fabsf(I) + fabsf(Q);
    const float on  = threshold * s.avg_amp;   // event while above this

    // replace the last 'gap' stored samples by a straight line from the
    // last good sample before them to the current (good) sample
    auto blank = [&](const uint32_t gap)
    {
      const uint32_t ia = (s.w - gap - 1u) & NB_MASK;  // last good sample
      const float aI = s.bufI[ia];
      const float aQ = s.bufQ[ia];
      const float stepI = (I - aI) / (float)(gap + 1u);
      const float stepQ = (Q - aQ) / (float)(gap + 1u);
      for (uint32_t p = 1u; p <= gap; p++)
      {
        const uint32_t idx = (s.w - (gap + 1u - p)) & NB_MASK;
        s.bufI[idx] = aI + stepI * (float)p;
        s.bufQ[idx] = aQ + stepQ * (float)p;
      }
    };

    switch (s.state)
    {
      case NB_IDLE:
      {
        if (amp > on)
        {
          s.state = NB_EVENT;
          s.run = 1u;
          s.peak = amp;
        }
        else
        {
          s.avg_amp += alpha * (amp - s.avg_amp);
        }
        break;
      }
      case NB_EVENT:
      {
        if (amp > on)
        {
          if (amp > s.peak) s.peak = amp;
          if (++s.run > MAX_BLANK_RUN)
          {
            s.state = NB_SIGNAL;             // too long for a pop
          }
        }
        else
        {
          // pop over. A pop only just over the threshold (a noise peak or
          // voice transient, most false triggers) is blanked exactly as
          // before: GUARD_PRE + the samples over the threshold. A pop well
          // over the threshold also gets 'extra' samples on BOTH sides, enough
          // for its ringing to decay back down to the threshold.
          const float over_db = 20.0f * log10f(s.peak / on);
          uint32_t extra = 0u;
          if (over_db > BIG_POP_DB)
          {
            extra = (uint32_t)((over_db - BIG_POP_DB) / RING_DB_PER_SAMPLE) + 1u;
            if (extra > GUARD_EXTRA_MAX) extra = GUARD_EXTRA_MAX;
          }
          if (extra == 0u)
          {
            blank(GUARD_PRE + s.run);        // this sample is clean: blank now
            s.avg_amp += alpha * (amp - s.avg_amp);
            s.state = NB_IDLE;
          }
          else
          {
            s.extra = extra;
            s.post = 1u;                     // this sample is the first trailing one
            s.state = NB_POST;
          }
        }
        break;
      }
      case NB_POST:
      {
        if (amp > on)
        {
          if (s.run + s.post + 1u <= MAX_BLANK_RUN)
          {
            // another pop inside the trailing guard: merge it
            s.run += s.post + 1u;
            s.post = 0u;
            if (amp > s.peak) s.peak = amp;
            s.state = NB_EVENT;
          }
          else
          {
            s.state = NB_SIGNAL;             // a dense train: treat as signal
          }
        }
        else if (s.post < s.extra)
        {
          s.post++;                          // still inside the trailing guard
        }
        else
        {
          // this sample is clean: blank pre-guard + pop + trailing guard
          blank(GUARD_PRE + s.extra + s.run + s.post);
          s.avg_amp += alpha * (amp - s.avg_amp);
          s.state = NB_IDLE;
        }
        break;
      }
      case NB_SIGNAL:
      {
        s.avg_amp += alpha * (amp - s.avg_amp);   // follow the signal
        if (amp <= on)
        {
          s.state = NB_IDLE;
        }
        break;
      }
    }

    // delay line: emit the sample from NB_LEN calls ago, store the current one
    const float outI = s.bufI[s.w];
    const float outQ = s.bufQ[s.w];
    s.bufI[s.w] = I;
    s.bufQ[s.w] = Q;
    s.w = (s.w + 1u) & NB_MASK;
    I = outI;
    Q = outQ;
  }

  // Audio impulse limiter: after the bandwidth filter, before the AGC.
  // Caps each sample at ratio x the recent average audio level, so a pop is
  // never much louder than the speech and can't knock the AGC gain down.
  static const float __not_in_flash_func(audio_limiter)(const float x, const uint8_t level)
  {
    static constexpr float ratio[6] = {0.0f, 6.0f, 5.0f, 4.0f, 3.0f, 2.5f};
    static constexpr float up = 0.0016f;  // level rises in ~20 ms
    static constexpr float down = 0.0001f;  // falls in ~320 ms, holds between words
    static constexpr float release = 0.02f;    // gain recovers in ~1.6 ms, covers the ringing
    static float avg = 1.0e-6f;
    static float gain = 1.0f;

    if (level == 0 || level > 5) return x;

    const float a = fabsf(x);
    const float lim = ratio[level] * avg;
    const float g = (a > lim) ? lim / a : 1.0f;

    // instant attack
    gain = (g < gain) ? g : gain + (1.0f - gain) * release;

    const float y = x * gain;
    const float d = fabsf(y) - avg;
    avg += d * ((d > 0.0f) ? up : down);

    // never let the reference reach zero
    if (avg < 1.0e-6f) avg = 1.0e-6f;
    return y;
  }

  static const int16_t __not_in_flash_func(process_ssb)(
    const int16_t in_i,
    const int16_t in_q,
    const uint32_t jnr_level,
    const uint8_t bw,
    const uint8_t nb_level,
    const uint8_t lim_level)
  {
    // set smeter ref for SSB
    meter_ref = AGC_AT_MINUS73_SSB;

    // remove DC
    float ii = FILTER::dc1f((float)in_i / 32768.0f);
    float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // noise blanker
    noise_blanker(ii,qq,nb_level);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // LPF
    const float audio_lpf = FILTER::bwf[bw](ssb);

    // HPF
    const float audio_raw = FILTER::hpf_200f(audio_lpf);

    // limiter
    const float audio_lim = audio_limiter(audio_raw,lim_level);

    // notch
    const float audio_notch = notch.process(audio_lim);

    // JNR
    const float audio_out = FILTER::jnr(audio_notch,jnr_level);

    // AGC returns 12 bit value
    return agc(audio_out * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_cw)(const int16_t in_i,const int16_t in_q,float &sig)
  {
    // set smeter ref for SSB
    meter_ref = AGC_AT_MINUS73_CW;

    // remove DC
    const float ii = FILTER::dc1f((float)in_i / 32768.0f);
    const float qq = FILTER::dc2f((float)in_q / 32768.0f);

    // phase shift IQ +/- 45
    const float p45 = FILTER::fap1f(ii);
    const float n45 = FILTER::fap2f(qq);

    // reject image
    const float ssb = p45 - n45;

    // BPF for CW
    sig = FILTER::bpf_700f(ssb);

    // AGC returns 12 bit value
    return agc(sig * 32768.0f);
  }

  static const int16_t __not_in_flash_func(process_am)(const int16_t in_i, const int16_t in_q, const uint32_t jnr_level)
  {
    // AM envelope detector at zero IF with carrier-referenced AGC.
    // The DC component of the envelope IS the carrier level A, so a slow
    // LPF of the envelope gives a gain reference that ignores modulation
    // entirely - no syllabic pumping - while still tracking QSB.

    // set smeter ref for SSB
    meter_ref = AGC_AT_MINUS73_AM;

    // AGC settings
    static constexpr float max_gain = 100.0f;

    // one-pole corner ~ alpha*fs/(2*pi)
    // = ~5 Hz at fs = 31.25 kHz;
    // scale for your RX sample rate
    static constexpr float agc_alpha = 0.001f; 

    // DAC amplitude at m = 1.0; leaves
    // ~+28% positive-peak headroom
    // below the 12-bit limit of 2047
    static constexpr float target = 1600.0f;

    // carrier level
    volatile static float agc_carrier = 0.0f;

    // channel filter, pre-detection: two instances of the SAME design
    // (identical coefficients, separate state per instance)
    const float ii = FILTER::lpf_fs8f_rx1((float)in_i / 32768.0f);
    const float qq = FILTER::lpf_fs8f_rx2((float)in_q / 32768.0f);

    // exact envelope at zero IF (single VSQRT.F32 on the RP2350 FPU)
    const float envelope = sqrtf(ii * ii + qq * qq);

    // carrier tracker: slow one-pole LPF of the envelope. The fmaxf
    // floor at envelope/2 is a fast-attack guard: a strong signal
    // appearing suddenly is caught to within 6 dB immediately instead
    // of blasting through for the LPF settling time. For m <= 1 the
    // floor never exceeds the true carrier, so it cannot cause pumping;
    // the slow tracker then converges the last few dB smoothly.
    const float agc_envelope = envelope * 32768.0f;
    agc_carrier += (agc_envelope - agc_carrier) * agc_alpha;
    if (agc_envelope > 4.0f * agc_carrier)
    {
      agc_carrier = agc_envelope * 0.5f;
    }
    // meter: x2 so a carrier reads the same as in SSB/CW, where the
    // phasing sum (p45 - n45) gives agc_peak = 2 x |IQ| for a tone
    // AM drives both detector variables: agc() isn't called in AM, and the
    // carrier tracker is already averaged, so peak and average are the same
    agc_peak    = agc_carrier * 2.0f;
    meter_level = agc_carrier * 2.0f;

    // gain from carrier level: m = 1.0 maps to 'target' at the DAC
    float gain = max_gain;
    if (agc_carrier > 1.0f)
    {
      gain = target / agc_carrier;
    }
    gain = fminf(gain, max_gain);

    // carrier removal, audio shaping, noise reduction as before
    const float audio_raw = FILTER::lpf_3000f(FILTER::dcf(envelope));
    const float audio_out = FILTER::jnr(audio_raw, jnr_level);
    return (int16_t)(audio_out * 32768.0f * gain);
  }
}

#endif