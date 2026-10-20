/*
 * RotaryAnalog.h -- single-pin analog rotary encoder decoder
 *
 * Drop-in replacement for Ben Buxton's Rotary.h when ENCA/ENCB have
 * been summed onto one ADC pin through a resistor network, so you
 * can free up a GPIO. Same public interface, so existing code that
 * does:
 *
 *     Rotary r = Rotary(PIN_ENCA, PIN_ENCB);
 *     ...
 *     unsigned char dir = r.process();
 *
 * only needs the construction line changed to:
 *
 *     RotaryAnalog r = RotaryAnalog(PIN_ENC_ADC);
 *
 * Assumed hardware (Vcc = 3.3V):
 *
 *     encoder common -----------------------------> 3.3V
 *     ENCA ----R----+
 *                   +---- ADC pin
 *     ENCB ----2R---+
 *                   |
 *                   R
 *                   |
 *                  GND
 *
 * With R = 10k this gives four levels (theory / measured on uDSR):
 *     neither closed : 0.00 V / 0.0 V  (code    0 @ 12-bit)
 *     ENCB only      : 1.10 V / 1.0 V  (code 1241)
 *     ENCA only      : 1.65 V / 1.6 V  (code 1986)
 *     both closed    : 1.98 V / 2.0 V  (code 2482)
 *
 * The thresholds are raw 12-bit ADC counts read straight from adc_hw,
 * so analogReadResolution() has no effect on them.
 *
 * WHY THE SETTLING FILTER (v2)
 * ----------------------------
 * With two digital pins, a quadrature encoder only ever changes ONE bit
 * at a time, and Rotary.h's state table relies on that. On a summed
 * analog pin, a contact does not open or close instantly: its
 * resistance sweeps between open and closed over microseconds to
 * milliseconds as the wiper makes and breaks. While it does, the node
 * voltage passes THROUGH the voltage bands in between. For example,
 * ENCA closing while ENCB is already closed takes the node from 1.0 V
 * to 2.0 V via the 1.6 V "ENCA only" band. Sampled at 31.25 kHz, the
 * decoder sees B-only -> A-only -> both, a two-bit jump that can't
 * physically happen. The full-step table treats that as noise and
 * resets, so the step is lost.
 *
 * Every detent has one rising and one falling transition that cross a
 * middle band. The resets hit CW harder than CCW, so the visible
 * symptom is CW missing almost entirely, CCW missing most steps, and
 * a "slow" feel.
 *
 * Fix: a decoded level must be seen SETTLE consecutive times before it
 * is passed to the state table. A level the voltage is only passing
 * through never lasts that long, and contact bounce is absorbed at the
 * same time. The default SETTLE = 16 calls is 0.5 ms when process() is
 * called once per 31.25 kHz DSP sample, which still keeps up with well
 * over 300 detents/s.
 *
 * Don't add a capacitor on the ADC node to "clean it up": that slows
 * the transitions and puts MORE samples in the in-between bands.
 */

#ifndef RotaryAnalog_h
#define RotaryAnalog_h

#include <Arduino.h>
#include "hardware/adc.h"

#define DIR_NONE 0x0
#define DIR_CW   0x10
#define DIR_CCW  0x20

class RotaryAnalog
{
  public:
    // th1/th2/th3 are ADC counts at the midpoints between adjacent
    // voltage levels (neither|B-only, B-only|A-only, A-only|both).
    // Defaults are the midpoints of the measured uDSR levels
    // (0.0 / 1.0 / 1.6 / 2.0 V -> 0.5 / 1.3 / 1.8 V).
    // settle = consecutive identical readings needed to accept a level.
    RotaryAnalog(const uint8_t adcPin,
      const uint16_t th1 = 620,
      const uint16_t th2 = 1613,
      const uint16_t th3 = 2234,
      const uint8_t settle = 16)
      : pin(adcPin), t1(th1), t2(th2), t3(th3), settle_count(settle),
        state(R_START), stable(PS_IDLE), candidate(PS_IDLE), count(0) {}

    void begin(void)
    {
      state = R_START;
      stable = PS_IDLE;
      candidate = PS_IDLE;
      count = 0;
      uint8_t mux = 0;
      switch (pin)
      {
        case 26: mux = 0; break;
        case 27: mux = 1; break;
        case 28: mux = 2; break;
        case 29: mux = 3; break;
      }
      adc_init();
      adc_gpio_init(pin);
      adc_select_input(mux);
      // start a conversion
      hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);
    }

    // call at a steady rate (once per DSP sample)
    const uint8_t process(void)
    {
      const uint16_t v = (uint16_t)adc_hw->result;
      hw_set_bits(&adc_hw->cs, ADC_CS_START_ONCE_BITS);

      // Which of the four voltage bands are we in? Expressed in the same
      // (1 = open, 0 = closed) sense Rotary.h gets from INPUT_PULLUP +
      // common-to-GND wiring: bit 1 = ENCB, bit 0 = ENCA. The state table
      // below is unmodified from the original full-step Rotary.h.
      uint8_t pinstate;
      if (v < t1)      pinstate = 0x3;  // neither closed
      else if (v < t2) pinstate = 0x1;  // ENCB only
      else if (v < t3) pinstate = 0x2;  // ENCA only
      else             pinstate = 0x0;  // both closed

      // settling filter: ignore any level that doesn't persist
      if (pinstate != candidate)
      {
        candidate = pinstate;
        count = 0;
        return DIR_NONE;
      }
      if (count < settle_count)
      {
        count++;
      }
      if (count < settle_count || candidate == stable)
      {
        return DIR_NONE;
      }
      stable = candidate;

      // Full-step state table (public domain, Ben Buxton's Rotary library)
      static const uint8_t ttable[7][4] =
      {
        /* R_START     */ { R_START,    R_CW_BEGIN,  R_CCW_BEGIN, R_START },
        /* R_CW_FINAL  */ { R_CW_NEXT,  R_START,     R_CW_FINAL,  DIR_CW  | R_START },
        /* R_CW_BEGIN  */ { R_CW_NEXT,  R_CW_BEGIN,  R_START,     R_START },
        /* R_CW_NEXT   */ { R_CW_NEXT,  R_CW_BEGIN,  R_CW_FINAL,  R_START },
        /* R_CCW_BEGIN */ { R_CCW_NEXT, R_START,     R_CCW_BEGIN, R_START },
        /* R_CCW_FINAL */ { R_CCW_NEXT, R_CCW_FINAL, R_START,     DIR_CCW | R_START },
        /* R_CCW_NEXT  */ { R_CCW_NEXT, R_CCW_FINAL, R_CCW_BEGIN, R_START },
      };

      state = ttable[state & 0xf][stable];
      return state & 0x30;
    }

  private:
    enum
    {
      R_START     = 0x0,
      R_CW_FINAL  = 0x1,
      R_CW_BEGIN  = 0x2,
      R_CW_NEXT   = 0x3,
      R_CCW_BEGIN = 0x4,
      R_CCW_FINAL = 0x5,
      R_CCW_NEXT  = 0x6,
    };
    static const uint8_t PS_IDLE = 0x3;   // neither contact closed

    uint8_t pin;
    uint16_t t1;
    uint16_t t2;
    uint16_t t3;
    uint8_t settle_count;
    uint8_t state;
    uint8_t stable;
    uint8_t candidate;
    uint8_t count;
};

#endif