/*
 * uDSR Version 1.1.250
 *
 * Copyright 2026 Ian Mitchell VK7IAN
 * Licenced under the GNU GPL Version 3
 *   (unless otherwise indicated)
 *
 * Libraries
 *
 *  TFT_eSPI2 in TFF_ePSI2 folder (based on https://github.com/Bodmer/TFT_eSPI)
 *
 * Filter Design
 *
 *  https://www.arc.id.au/FilterDesign.html
 *
 * Build:
 *  Seeed XIAO RP2350
 *  CPU Speed: 250Mhz
 *  Optimize: -O3
 *  USB Stack: No USB
 *  Flash Size: 2MB (no FS)
 *
 * Some history
 *  0.1.250 start with uDST, uPDCR, and iqSDR40
 *  0.2.250 firmware update from menu
 *  0.3.250 smeter fixes
 *  0.4.250 set smeter by mode
 *  0.5.250 dithered waterfall
 *  0.6.250 waterfall speed
 *  0.7.250 improved noise blanker
 *  1.0.250 release
 *  1.1.250 limiter (audio blanker)
 */

//#define DEBUG_LED
//#define DEBUG_METER_DBM
//#define DEBUG_DDC_SAMPLE
//#define DEBUG_AGC_SAMPLE
//#define SPI_LOOPBACK_TEST
// uncomment and connect D1 and D2
// using a 220 Ohm resistor
// to test asyncSPI functions

#include <EEPROM.h>
#include "Tiny4kWire1.h"
#include "asyncSPI.h"
#include "util.h"
#include "RotaryAnalog.h"
#include "spectrum.h"
#include "dsp.h"
#include "menu.h"
#include "cwdecode1.h"
#include "cwdecode2.h"
#include "hardware/pwm.h"
#include "hardware/vreg.h"
#include "pico/bootrom.h"

#define VERSION_STRING "  V1.1."
#define MENU_TIMEOUT 5000u
#define POPUP_TIMEOUT 5000u
#define BAND_80M 0
#define BAND_40M 1
#define BAND_30M 2
#define BAND_20M 3
#define BAND_17M 4
#define BAND_15M 5
#define BAND_12M 6
#define BAND_10M 7
#define BAND_SWL 8
#define BAND_MIN BAND_80M
#define BAND_MAX BAND_SWL
#define NUM_BANDS 9
#define DEFAULT_FREQUENCY 7100000ul
#define DEFAULT_BAND BAND_40M
#define DEFAULT_MODE MODE_LSB
#define DEFAULT_STEP 1000ul
#define DEFAULT_BANDWIDTH 3ul
#define NOTCH_LOW 300ul
#define NOTCH_HIGH 2600ul
#define DEFAULT_NOTCH 1000ul
#define DEFAULT_SPEED 1ul
#define BUTTON_LONG_PRESS_TIME 800ul
#define SPECTRUM_LEVEL_MIN -4
#define SPECTRUM_LEVEL_MAX 5
#define FPGA_CLK 81'000'000ULL // mine measured: 80'999'683
//#define FPGA_CLK 80'999'683

#define PIN_ENC     A0  // 26 analog encode pin (ADC0)
#define PIN_ENCBUT  D7  // 1 button on rotary
#define PIN_SDA     D4  // 6 I2C (this is Wire1)
#define PIN_SCL     D5  // 7 I2C (this is Wire1)
#define PIN_IQSCK   D6  // 0 async SPI
#define PIN_IQMOSI  D1  // 27 async SPI
#define PIN_IQMISO  D2  // 28 async SPI
#define PIN_IQCS    D3  // 5 async SPI
#define PIN_DR      D9  // 4 FPGA Data Ready
#define PIN_AUPWML  D10 // 3 audio PWM low
#define PIN_AUPWMH  D8  // 2 audio PWM high

#define JNR_OFF        0u
#define JNR_LEVEL1     1u
#define JNR_LEVEL2     2u
#define JNR_LEVEL3     3u
#define NB_OFF         0u
#define NB_LEVEL1      1u
#define NB_LEVEL2      2u
#define NB_LEVEL3      3u
#define NB_LEVEL4      4u
#define NB_LEVEL5      5u
#define LIMITER_OFF    0u
#define LIMITER_LEVEL1 1u
#define LIMITER_LEVEL2 2u
#define LIMITER_LEVEL3 3u
#define LIMITER_LEVEL4 4u
#define LIMITER_LEVEL5 5u
#define POS_MENU_X     25u

// two buffers of 1024 FFT bins
#define MAX_IQ_SAMPLES  2048
#define SPECTRUM_BUFFER 1024

// EEPROM boot loader flag
#define BOOTLOADER_REQUEST  0x42

static_assert((MAX_IQ_SAMPLES & (MAX_IQ_SAMPLES - 1)) == 0, "MAX_IQ_SAMPLES must be a power of 2");

// just keep same error codes as MBPTRX
#define ERROR_SYSCLOCK 3u
#define ERROR_FPGA     4u
#define ERROR_OLED     6u

enum spectrum_layout_t
{
  LAYOUT_METER,
  LAYOUT_SPECTRUM,
  LAYOUT_CW,
  LAYOUT_WATERFALL
};

enum radio_mode_t
{
  MODE_LSB,
  MODE_USB,
  MODE_CWL,
  MODE_CWU,
  MODE_AM
};

volatile static struct
{
  int32_t tune;
  uint32_t tuning_word;
  uint32_t step;
  uint32_t frequency;
  uint32_t band;
  uint32_t cwdecode;
  uint32_t jnrlevel;
  uint32_t nblevel;
  uint32_t limiter;
  uint32_t bandwidth;
  uint32_t notch;
  uint32_t update;
  uint32_t waterspeed;
  radio_mode_t mode;
  bool menu_active;
  bool mode_auto;
  bool notch_enable;
  bool mute;
  bool spectrum;
  bool waterfall;
  int8_t level[NUM_BANDS];
  int8_t slevel[NUM_BANDS];
}
radio =
{
  0l,
  0ul,
  DEFAULT_STEP,
  DEFAULT_FREQUENCY,
  DEFAULT_BAND,
  0l,
  JNR_OFF,
  NB_OFF,
  LIMITER_OFF,
  DEFAULT_BANDWIDTH,
  DEFAULT_NOTCH,
  0ul,
  DEFAULT_SPEED,
  DEFAULT_MODE,
  false,
  true,
  false,
  true,
  true,
  false,
  {0,0,0,0,0,0,0,0,0},
  {0,0,0,0,0,0,0,0,0}
};

static struct
{
  const uint32_t lo;
  const uint32_t hi;
  uint32_t frequency;
}
bands[] =
{
  { 3500000UL,  3800000UL,  3600000UL},
  { 7000000UL,  7300000UL,  7100000UL},
  {10100000UL, 10150000UL, 10120000UL},
  {14000000UL, 14350000UL, 14200000UL},
  {18068000UL, 18168000UL, 18100000UL},
  {21000000UL, 21450000UL, 21200000UL},
  {24890000UL, 24990000UL, 24900000UL},
  {28000000UL, 29700000UL, 28500000UL},
  {  500000UL, 30000000UL,  5000000UL}
};

volatile static struct
{
  uint32_t frequency;
}
save_data[] =
{
  { 3600000UL},
  { 7100000UL},
  {10120000UL},
  {14200000UL},
  {18100000UL},
  {21200000UL},
  {24900000UL},
  {28500000UL},
  { 5000000UL}
};

volatile static struct
{
  uint32_t usage;
  uint32_t count;
} cpu = {0,0};

// For RP2040/RP2350 multicore: Allocates a dedicated 8KB stack for Core 1
// By default, Core 0 and Core 1 split a single 8KB stack (4KB each)
// Setting this to true prevents Core 1 from causing stack overflow crashes
bool core1_separate_stack = true;

RotaryAnalog r = RotaryAnalog(PIN_ENC);
AsyncSPI iqSPI(pio0, 0, PIN_IQCS, PIN_IQSCK, PIN_IQMOSI, PIN_IQMISO, 6'000'000.0f);

// rotary is processed on core 1
// frequency is pocessed on core 0
// tuning word makes its way back to core 1
// so it can be transfered to the FPGA
auto_init_mutex(rotary_mutex);
auto_init_mutex(cw_decode_mutex);

volatile static int32_t debug_value_1 = 0;
volatile static int32_t debug_value_2 = 0;
volatile static uint32_t audio_pwm = 0;
volatile static uint32_t iq_sample_p = 0;
volatile static int16_t adc_data_i[MAX_IQ_SAMPLES] = {0};
volatile static int16_t adc_data_q[MAX_IQ_SAMPLES] = {0};
volatile static bool setup_complete = false;
volatile static bool set_spectrum_level = false;
volatile static bool adj_spectrum_level = false;
volatile static bool adj_notch_filter = false;
volatile static char cw_decode_buf[32] = "";
volatile static char sz_version[16] = "";
static uint8_t magnitude[1024] = {0};

static void error_stop(const uint32_t err_code)
{
  // self-test error code
  // LED is avtive low
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  for (;;)
  {
    for (uint32_t i=0;i<err_code;i++)
    {
      digitalWrite(LED_BUILTIN,LOW);
      delay(50);
      digitalWrite(LED_BUILTIN,HIGH);
      delay(250);
    }
    delay(1000);
  }
}

static void run_spi_loopback_test(void)
{
  // connect D1 to D2 through a 220 Ohm resistor!
  // --- test parameters ---
  static const uint32_t XFER_GAP_US = 30;     // let each 32-bit transfer finish (~6 us) + margin
  static const uint32_t WINDOW = 16;          // rolling history depth for connect/disconnect decision
  static const uint32_t MATCH_THRESHOLD = 14; // >=14 of last 16 must match to call it "connected"

  // Test vectors chosen to expose specific failures:
  //   0x00000000 / 0xFFFFFFFF : stuck-high / stuck-low bits
  //   0xFFFF0000 / 0x0000FFFF : I-half vs Q-half not swapped
  //   0x80000001              : MSB and LSB positions specifically
  //   alternating + arbitrary : adjacent-bit and general integrity
  static const uint32_t patterns[] =
  {
    0x00000000u, 0xFFFFFFFFu, 0xA5A5A5A5u, 0x5A5A5A5Au,
    0x12345678u, 0x80000001u, 0xDEADBEEFu, 0x0000FFFFu,
    0xFFFF0000u
  };
  static const uint32_t NPAT = sizeof(patterns) / sizeof(patterns[0]);

  // Pull MISO down so a DISCONNECTED input reads a stable 0 (not floating
  // noise). The PIO already claimed the pin; the pad pull bits are independent
  // of the function select, so this is safe to set afterward.
  gpio_pull_down(PIN_IQMISO);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH);

  // Rolling history of recent match results (1 = matched, 0 = mismatch).
  uint8_t hist[WINDOW] = {0};
  uint32_t hist_pos = 0;
  uint32_t match_in_win = 0;
  uint32_t prev_sent  = 0;
  bool have_prev = false;
  uint32_t idx = 0;
  uint32_t pass_total = 0;
  uint32_t fail_total = 0;
  uint32_t last_led_ms = 0;

  for (;;)
  {
    const uint32_t sent = patterns[idx % NPAT];
    const uint32_t got = iqSPI.doTransfer(sent);
    sleep_us(XFER_GAP_US); // let the transfer complete before next call

    // Pipeline: 'got' is the loopback of the PREVIOUS word, so compare to prev_sent.
    if (have_prev)
    {
      const uint8_t matched = (got == prev_sent) ? 1u : 0u;
      if (matched) pass_total++; else fail_total++;

      // slide the rolling window
      match_in_win -= hist[hist_pos];
      hist[hist_pos] = matched;
      match_in_win += matched;
      hist_pos = (hist_pos + 1) % WINDOW;
    }
    prev_sent = sent;
    have_prev = true;
    idx++;

    // "connected" if most of the recent window matched. The rolling window
    // tolerates the one ambiguous case (sending 0x00000000 while disconnected
    // also reads 0 via the pulldown) because the surrounding nonzero patterns
    // dominate the decision.
    const bool connected = (match_in_win >= MATCH_THRESHOLD);

    // live LED: fast blink = passing, slow blink = failing/disconnected
    const uint32_t now = millis();
    const uint32_t period = connected ? 100u : 600u;
    if (now - last_led_ms >= period)
    {
      digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
      last_led_ms = now;
    }
  }
}

// Verify the FPGA is alive by checking data_ready is toggling near 31 kHz.
// Returns true if a plausible pulse train is present, false otherwise.
// Call from setup(); on false, invoke error_stop(ERROR_FPGA).
static const bool check_fpga_data_ready(void)
{
  // data_ready is ~31.25 kHz, period ~32 us,
  // ~16 us high / ~16 us low. We don't measure exact frequency -- we just
  // confirm it's toggling at roughly the right rate, which separates "alive"
  // from "dead/disconnected/not-clocking"

  // Count edges over a fixed window and check the count is in a sane band.
  // Over 10 ms at 31.25 kHz we expect ~312 full cycles = ~625 edges.
  // exact clock doesn't matter; we're proving "alive", not measuring Hz.

  static const uint32_t window_us = 10000; // 10 ms observation
  static const uint32_t expect_edges = 625; // ~ at 31.25 kHz
  static const uint32_t min_edges = expect_edges / 2; // ~half rate -> still "alive"
  static const uint32_t max_edges = expect_edges * 2; // ~double rate -> sanity ceiling

  pinMode(PIN_DR, INPUT);
  uint32_t edges = 0;
  int last = digitalRead(PIN_DR);
  const absolute_time_t t_end = make_timeout_time_us(window_us);
  while (!time_reached(t_end))
  {
    const int now = digitalRead(PIN_DR);
    if (now != last)
    {
      edges++;
      last = now;
    }
  }
  return (edges >= min_edges) && (edges <= max_edges);
}

static __attribute__((always_inline)) inline void mute(void)
{
  // set a flag but mute in loop1
  radio.mute = true;
}

static __attribute__((always_inline)) inline void unmute(void)
{
  // clear flag but unmute in loop1
  radio.mute = false;
}

static void save_settings(void)
{
  static const uint32_t key = 0x12345678ul;
  mute();
  delay(100);
  EEPROM.begin(256);
  uint32_t i = 0;
  EEPROM.put(i,key);                       i += sizeof(uint32_t);
  EEPROM.put(i,(uint32_t)radio.jnrlevel);  i += sizeof(uint32_t);
  EEPROM.put(i,(uint32_t)radio.bandwidth); i += sizeof(uint32_t);
  EEPROM.put(i,(uint32_t)radio.update);    i += sizeof(uint32_t);
  for (uint32_t j=0;j<NUM_BANDS;j++)
  {
    EEPROM.put(i,(uint32_t)radio.level[j]);
    i += sizeof(uint32_t);
  }
  EEPROM.end();
  unmute();
}

static void restore_settings(void)
{
  uint32_t key = 0 ;
  EEPROM.begin(256);
  EEPROM.get(0,key);
  if (key==0x12345678)
  {
    uint32_t i = sizeof(uint32_t);
    uint32_t data32 = 0;
    EEPROM.get(i,data32); radio.jnrlevel  = data32; i += sizeof(uint32_t);
    EEPROM.get(i,data32); radio.bandwidth = data32; i += sizeof(uint32_t);
    EEPROM.get(i,data32); radio.update    = data32; i += sizeof(uint32_t);
    for (uint32_t j=0;j<NUM_BANDS;j++)
    {
      EEPROM.get(i,data32);
      i += sizeof(uint32_t);
      radio.level[j] = (int8_t)data32;
      if (radio.level[j]<SPECTRUM_LEVEL_MIN || radio.level[j]>SPECTRUM_LEVEL_MAX)
      {
        radio.level[j] = 0;
      }
      radio.slevel[j] = radio.level[j];
    }
  }
  if (radio.jnrlevel>3)
  {
    radio.jnrlevel = JNR_OFF;
  }
  if (radio.bandwidth<1ul || radio.bandwidth>5ul)
  {
    radio.bandwidth = DEFAULT_BANDWIDTH;
  }
  EEPROM.end();
}

static void set_frequency(void)
{
  constexpr uint64_t TWO32 = 1ULL << 32; // 2^32
  constexpr uint64_t DENOM = FPGA_CLK * 100ULL; // 80e6 Hz * 100 = 8e9
  const int32_t cwoffset = 700;
  int32_t rx_cwoffsetw = 0;
  switch (radio.mode)
  {
    case MODE_CWL: rx_cwoffsetw = +cwoffset; break;
    case MODE_CWU: rx_cwoffsetw = -cwoffset; break;
  }
  const uint64_t f_centiHz = 100ull * (radio.frequency+rx_cwoffsetw);
  // numerator = f_centiHz * 2^32  -- check this can't overflow 64 bits:
  //   max f_centiHz for 30 MHz = 3,000,000,000 (3e9)
  //   3e9 * 2^32 ~= 1.29e19, and uint64 max ~= 1.84e19 -> fits, but tightly.
  // Round-to-nearest: add half the denominator before dividing.
  const uint64_t num = f_centiHz * TWO32;
  radio.tuning_word = (uint32_t)((num + DENOM/2) / DENOM);
  return ;
}

// ---------------------------------------------------------------------------
// OLED start-up hardening
//
// Symptom seen (mostly from cold): FPGA fine, no error flashes, but the OLED
// stays dark. No error flash means the OLED ACKed on I2C, so it answered but
// did not act on (or later lost) its init sequence, most likely because its
// internal reset/charge pump had not settled when the commands arrived.
//
// Defences, cheapest first:
//   1. i2c_bus_clear(): free SDA if a device was left mid-byte by a power blip
//   2. extra settling time before the first init (OLED_POWERUP_MS)
//   3. retry the probe a few times before declaring ERROR_OLED
//   4. send the full init sequence twice, 100 ms apart
//   5. send it once more after the 4 s splash; by then the OLED has had
//      >5 s, so even a slow cold start is caught before the main display
// ---------------------------------------------------------------------------
#define OLED_POWERUP_MS 500u
#define OLED_RETRIES    5u

static void i2c_bus_clear(void)
{
  // a device left mid-byte can hold SDA low: clock SCL until it
  // lets go, then generate a STOP (SDA low -> high while SCL high)
  pinMode(PIN_SDA,INPUT_PULLUP);
  pinMode(PIN_SCL,INPUT_PULLUP);
  delayMicroseconds(10);
  for (uint32_t i=0;i<9 && digitalRead(PIN_SDA)==LOW;i++)
  {
    digitalWrite(PIN_SCL,LOW);
    pinMode(PIN_SCL,OUTPUT);
    delayMicroseconds(5);
    pinMode(PIN_SCL,INPUT_PULLUP);
    delayMicroseconds(5);
  }
  digitalWrite(PIN_SDA,LOW);
  pinMode(PIN_SDA,OUTPUT);
  delayMicroseconds(5);
  pinMode(PIN_SDA,INPUT_PULLUP);
  delayMicroseconds(5);
}

static void i2c_start(void)
{
  // (re)start Wire1 on a clean bus; safe to call more than once
  Wire1.end();
  i2c_bus_clear();
  Wire1.setSDA(PIN_SDA);
  Wire1.setSCL(PIN_SCL);
  Wire1.setClock(400000ul);
  Wire1.begin();
}

static void oled_init(void)
{
  // full init sequence plus our settings, bright and rotated
  // note: include file declares the oled object
  oled.begin(128,32,sizeof(tiny4koled_init_128x32br),tiny4koled_init_128x32br);
  oled.enableChargePump();
  oled.setRotation(1);
  oled.setInternalIref(true);
  oled.clear();
  oled.on();
}

static const bool oled_start(void)
{
  for (uint32_t attempt=0;attempt<OLED_RETRIES;attempt++)
  {
    oled_init();
    if (oledFound())
    {
      // second pass in case the first landed while the OLED was still waking
      delay(100);
      oled_init();
      return true;
    }
    delay(200);
    i2c_start();
  }
  return false;
}

static void oled_reinit(void)
{
  // Late re-init, called after the splash. The init sequence resets the
  // controller's display start line to frame 0, but Tiny4kOLED's own
  // double-buffer state is unchanged. Two switchFrame() calls leave that state
  // exactly as it was (each toggles it), and the second one re-sends the
  // display frame the library expects, so hardware and library agree again.
  // The clear()s blank both frames on the way.
  oled_init();
  oled.clear();
  oled.switchFrame();
  oled.clear();
  oled.switchFrame();
}

void setup(void)
{
  for (uint8_t i=0;i<sizeof(cw_decode_buf);i++)
  {
    cw_decode_buf[i] = '\0';
  }

  // run UI on core 0
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,LOW);
  delay(1000);
  digitalWrite(LED_BUILTIN,HIGH);
  vreg_set_voltage(VREG_VOLTAGE_1_30);
  const uint32_t clksys = frequency_count_khz(CLOCKS_FC0_SRC_VALUE_CLK_SYS);
  // frequency_count_khz() isn't accurate!
  if (clksys < (250000ul - 5ul) || clksys > (250000ul + 5ul))
  {
    // trap the wrong system clock
    error_stop(ERROR_SYSCLOCK);
  }
  pinMode(PIN_ENCBUT,INPUT_PULLUP);
  pinMode(PIN_DR,INPUT);

  // init critical radio defaults
  radio.frequency = DEFAULT_FREQUENCY;
  radio.band = DEFAULT_BAND;
  radio.mode = DEFAULT_MODE;
  restore_settings();

  // check for bootloader request
  if (radio.update == BOOTLOADER_REQUEST)
  {
    radio.update = 0;
    save_settings();
    delay(100);
    reset_usb_boot(0, 0);
    for (;;);
  }

#ifndef SPI_LOOPBACK_TEST
  if (!check_fpga_data_ready())
  {
    // FPGA not present
    error_stop(ERROR_FPGA);
  }
#endif

  // high speed I2C, on a cleared bus
  i2c_start();

  // set up audio out PWM
  // 250,000,000 / 64 = 3,906,250
  gpio_set_function(PIN_AUPWMH,GPIO_FUNC_PWM);
  gpio_set_function(PIN_AUPWML,GPIO_FUNC_PWM);
  audio_pwm = pwm_gpio_to_slice_num(PIN_AUPWML);
  pwm_set_wrap(audio_pwm,63);
  pwm_set_both_levels(audio_pwm,31,0);
  pwm_set_enabled(audio_pwm,true);

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,LOW);
    delay(10);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(250);
  }
#endif

  // set default frequency
  set_frequency();

#ifdef SPI_LOOPBACK_TEST
  run_spi_loopback_test();
#endif

  // init rotary
  r.begin();

  // notch init and not enabled
  // default frequency
  // bandwidth and depth (dB)
  DSP::notch.init(31250.0f);
  DSP::notch.set((float)DEFAULT_NOTCH, 100.0f, 30.0f);
  DSP::notch.enable(false);

#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  digitalWrite(LED_BUILTIN,HIGH);
  for (int i=0;i<2;i++)
  {
    digitalWrite(LED_BUILTIN,LOW);
    delay(10);
    digitalWrite(LED_BUILTIN,HIGH);
    delay(250);
  }
#endif

  // OLED: extra settling time from cold, retried probe, double init
  delay(OLED_POWERUP_MS);
  if (!oled_start())
  {
    error_stop(ERROR_OLED);
  }
  oled.switchRenderFrame();

  // intro screen
  char sz_clksys[16] = "";
  memset(sz_clksys,0,sizeof(sz_clksys));
  memset((char*)sz_version,0,sizeof(sz_version));
  ultoa(clksys,sz_clksys,10);
  sz_clksys[3] = '\0';
  strcpy((char*)sz_version,VERSION_STRING);
  strcat((char*)sz_version,sz_clksys);
  oled.clear();
  oled.setFont(FONT8X16);
  oled.setCursor(50,0);
  oled.print(" uDSR");
  oled.setFont(FONT6X8);
  oled.setCursor(0,3);
  oled.print((const char*)sz_version);
  oled.setCursor(90,3);
  oled.print("VK7IAN");
  oled.setFont(FONT6X8);
  oled.switchFrame();

  // splash delay
  delay(4000);

  // belt and braces: re-send the OLED init now that it has had >5 s to wake,
  // in case the earlier init was missed (blank splash, but the radio display
  // still comes up)
  oled_reinit();

  // unmute
  unmute();

  setup_complete = true;
}

void setup1(void)
{
  // run DSP on core 1
  // only go to loop1 when setup() has completed
  while (!setup_complete)
  {
    tight_loop_contents();
  }
#ifdef DEBUG_LED
  pinMode(LED_BUILTIN,OUTPUT);
  for (int i=0;i<5;i++)
  {
    digitalWrite(LED_BUILTIN,HIGH);
    delay(100);
    digitalWrite(LED_BUILTIN,LOW);
    delay(250);
  }
#endif
}

// ---------------------------------------------------------------------------
// Display layout: 128x32 OLED, FONT6X8 = 21 character columns x 4 rows.
//
// NOTE: Tiny4kOLED setCursor(x,y) takes x in PIXELS (0..127) and y in 8-pixel
// PAGES (0..3). It is not character columns. text_at() converts a character
// column to pixels, so every position below is in characters.
//
//            0         1         2
//            012345678901234567890
//   row 0:   LSB  7.100.000   1000      mode, frequency (10 chars), step (6, right-aligned)
//   row 1:   40M 2400 NB3 J3 T 99%      band, bandwidth, NB, JNR, notch, CPU
//            (while adjusting spectrum level / notch, that text replaces row 1)
//   row 2/3: spectrum, waterfall, CW decode or S-meter dial: see the
//            "Spectrum layouts" block below (debug values overlay row 3)
//
//   Waterfall layout replaces the row 1 status text with an 8-pixel
//   spectrum (the adjust text still takes row 1 while adjusting).
//   radio.spectrum / radio.waterfall are never changed by the layout logic,
//   so turning CW decode off brings back whatever was set before.
//
// Fields that are off (NB, JNR, notch) show zero or left blank.
// ---------------------------------------------------------------------------
#define FONT_W     6u
#define LCD_COLS   21u
#define COL_MODE   0u
#define COL_FREQ   4u
#define COL_STEP   15u
#define COL_BAND   0u
#define COL_BW     4u
#define COL_NB     9u
#define COL_JNR    13u
#define COL_NOTCH  16u
#define COL_CPU    18u

static inline void text_at(const uint8_t col,const uint8_t row)
{
  oled.setCursor(col*FONT_W,row);
}

static void text_centre(const char *s,const uint8_t row)
{
  const uint32_t len = strlen(s);
  text_at(len<LCD_COLS?(LCD_COLS-len)/2u:0u,row);
  oled.print(s);
}

// ---------------------------------------------------------------------------
// Spectrum layouts. Exactly one is chosen each frame (spectrum_layout()):
//
//   LAYOUT_CW         CW decode on (and mode CWL/CWU), whatever else is set:
//                       row 2 = decoded text, row 3 = 8-pixel spectrum
//   LAYOUT_METER      spectrum off (radio.spectrum is the master switch):
//                       rows 2-3 = S-meter dial; radio.waterfall is ignored
//   LAYOUT_WATERFALL  spectrum on, waterfall on:
//                       row 1 = 8-pixel spectrum, rows 2-3 = waterfall
//                       (row 1 status text is hidden; adjust text still shows)
//   LAYOUT_SPECTRUM   spectrum on, waterfall off:
//                       rows 2-3 = 16-pixel spectrum
//
// In every spectrum the top pixel row carries the S-meter bar.
// Bytes are SSD1306 page format: bit 7 = BOTTOM pixel, bit 0 = TOP pixel.
// The three spectrum functions are deliberately separate (some duplication)
// so each layout can be read on its own.
// ---------------------------------------------------------------------------
static const bool adjust_active(void);   // defined with show_adj_settings()

// CW decode layout: decoded text on row 2, compact spectrum on row 3
static const bool cw_decode_active(void)
{
  return (radio.cwdecode!=0) && (radio.mode==MODE_CWU || radio.mode==MODE_CWL);
}

static const spectrum_layout_t spectrum_layout(void)
{
  if (cw_decode_active()) return LAYOUT_CW;        // decode needs its own rows
  if (!radio.spectrum)    return LAYOUT_METER;     // spectrum off: meter dial
  if (radio.waterfall)    return LAYOUT_WATERFALL; // waterfall is a spectrum option
  return LAYOUT_SPECTRUM;
}

// one value per display column (128): the max of 8 adjacent FFT bins, 0..31
static void spectrum_columns(uint8_t col[128])
{
  const uint8_t shift = radio.frequency<3'000'000?1:0;
  for (uint32_t i=0;i<128;i++)
  {
    uint8_t max_value = 0;
    for (uint32_t j=0;j<8;j++)
    {
      const uint8_t m = magnitude[i*8+j];
      if (m>max_value) max_value = m;
    }
    col[i] = max_value >> shift;
  }
}

// ---- LAYOUT_SPECTRUM: 16-pixel spectrum on rows 2 and 3 ----
static void show_spectrum_16(const uint32_t signal)
{
  static const uint8_t upper[16] =   // page 2, levels 8..15 grow upward
  {
    0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u, 0x00u,
    0x80u, 0xc0u, 0xe0u, 0xf0u, 0xf8u, 0xfcu, 0xfeu, 0xffu
  };
  static const uint8_t lower[16] =   // page 3, levels 0..7 grow upward
  {
    0x80u, 0xc0u, 0xe0u, 0xf0u, 0xf8u, 0xfcu, 0xfeu, 0xffu,
    0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu, 0xffu
  };
  static uint8_t col[128];
  static uint8_t buf[256];           // [0..127] = page 2, [128..255] = page 3
  spectrum_columns(col);
  for (uint32_t i=0;i<128;i++)
  {
    const uint8_t v = (col[i]>15u)?15u:col[i];
    const uint8_t sbar = (i<signal)?0x01u:0x00u;   // S-meter on the top line
    buf[i]     = upper[v] | sbar;
    buf[i+128] = lower[v];
  }
  oled.bitmap(0, 2, 128, 4, buf);
}

// ---- LAYOUT_CW: 8-pixel spectrum on row 3 (row 2 is the decoded text) ----
static void show_spectrum_cw(const uint32_t signal)
{
  // bar heights 1..7 px from the bottom, top pixel kept for the S-meter
  static const uint8_t bars[7] = {0x80u, 0xc0u, 0xe0u, 0xf0u, 0xf8u, 0xfcu, 0xfeu};
  static uint8_t col[128];
  static uint8_t buf[128];
  spectrum_columns(col);
  for (uint32_t i=0;i<128;i++)
  {
    const uint8_t v = (col[i]>15u)?15u:col[i];
    const uint8_t sbar = (i<signal)?0x01u:0x00u;
    buf[i] = bars[(v*7u)/16u] | sbar;              // 0..15 -> 7 heights
  }
  oled.bitmap(0, 3, 128, 4, buf);
}

// ---- LAYOUT_WATERFALL: 8-pixel spectrum on row 1, waterfall on rows 2-3 ----
//
// The waterfall is 128 columns x 16 pixel rows, newest line at the top,
// scrolling down. Each column is held as a uint16_t: bit 0 = top pixel
// (page 2 bit 0) ... bit 15 = bottom pixel (page 3 bit 7), so scrolling down
// is just col <<= 1 and the new pixel goes in bit 0.
//
// Grey levels on a 1-bit display come from ordered (Bayer 4x4) dithering:
// a pixel is lit when its level beats a threshold that repeats every 4
// columns and every 4 lines, so level 4/16 lights 1 pixel in 4, 8/16 half
// of them, and so on. Because the line counter steps the pattern, a steady
// signal shows as an even texture rather than vertical stripes.
//
// Contrast: the noise floor of each line (its median) is subtracted, so
// noise sits black and signals stand out without a manual level setting.
//
// Near the floor, Bayer lights the SAME tile position everywhere, so faint
// noise shows as a regular grid. Levels up to WF_RANDOM_BELOW therefore use
// a random threshold instead: the same average density, but scattered like
// real noise. Brighter levels keep Bayer for a smooth, even signal texture.
// 0 = Bayer everywhere (old look), 16 = random everywhere.
#define WF_LINE_MS_SLOW   150u // one new line every 150 ms -> 16 lines = 2.4 s
#define WF_LINE_MS_MEDIUM 60u  // one new line every 100 ms -> 16 lines = 1.0 s
#define WF_LINE_MS_FAST   32u  // one new line every 75 ms -> 16 lines = 0.5 s
#define WF_CONTRAST       2u   // log steps above the floor -> dither levels
#define WF_NOISE_FILL     1u   // dither level (out of 16) at the noise floor: 0 = black noise
#define WF_RANDOM_BELOW   4u   // levels <= this use random dither (breaks up the grid)

// small fast pseudo-random generator (xorshift32) for the dither
static inline uint32_t wf_random(void)
{
  static uint32_t s = 0x2545f491u;
  s ^= s << 13;
  s ^= s >> 17;
  s ^= s << 5;
  return s;
}

static void show_waterfall(const uint32_t signal)
{
  static const uint8_t bars[7] = {0x80u, 0xc0u, 0xe0u, 0xf0u, 0xf8u, 0xfcu, 0xfeu};
  static const uint8_t bayer4[4][4] =
  {
    { 0u,  8u,  2u, 10u},
    {12u,  4u, 14u,  6u},
    { 3u, 11u,  1u,  9u},
    {15u,  7u, 13u,  5u}
  };
  static uint16_t wf[128] = {0};   // the waterfall image, one word per column
  static uint8_t  peak[128] = {0}; // column peak since the last line
  static uint32_t line = 0;        // line counter, steps the dither pattern
  static uint32_t next_line_ms = 0;
  static uint8_t  col[128];
  static uint8_t  buf[128];

  uint32_t WF_LINE_MS = 100;
  switch (radio.waterspeed)
  {
    case 0: WF_LINE_MS = WF_LINE_MS_SLOW;   break;
    case 1: WF_LINE_MS = WF_LINE_MS_MEDIUM; break;
    case 2: WF_LINE_MS = WF_LINE_MS_FAST;   break;
  }

  spectrum_columns(col);

  // row 1: 8-pixel spectrum with the S-meter on its top line
  for (uint32_t i=0;i<128;i++)
  {
    const uint8_t v = (col[i]>15u)?15u:col[i];
    const uint8_t sbar = (i<signal)?0x01u:0x00u;
    buf[i] = bars[(v*7u)/16u] | sbar;
  }
  if (!adjust_active())                // adjust text owns row 1 meanwhile
  {
    oled.bitmap(0, 1, 128, 2, buf);
  }

  // hold the peak of each column between waterfall lines, so short
  // signals (CW dits, FT8 tones) are not lost between lines
  for (uint32_t i=0;i<128;i++)
  {
    if (col[i]>peak[i]) peak[i] = col[i];
  }

  // time for a new line?
  const uint32_t now = millis();
  if ((int32_t)(now - next_line_ms) >= 0)
  {
    next_line_ms = now + WF_LINE_MS;

    // noise floor = median column level (histogram of the 0..31 levels)
    uint8_t hist[32] = {0};
    for (uint32_t i=0;i<128;i++) hist[peak[i]&31u]++;
    uint32_t floor_level = 0;
    uint32_t count = hist[0];
    while (count<64u && floor_level<31u)   // 64 = half of the 128 columns
    {
      floor_level++;
      count += hist[floor_level];
    }

    // scroll down one pixel and dither the new line into the top row
    const uint8_t *threshold = bayer4[line & 3u];
    for (uint32_t i=0;i<128;i++)
    {
      int32_t level = ((int32_t)peak[i] - 
        (int32_t)floor_level) * (int32_t)WF_CONTRAST + 
        (int32_t)WF_NOISE_FILL;
      if (level<0) level = 0;
      if (level>16) level = 16;
      // random threshold 0..15 near the floor, Bayer above it
      const uint32_t t = ((uint32_t)level <= WF_RANDOM_BELOW) ?
        (wf_random() >> 28) : threshold[i & 3u];
      const uint16_t pixel = ((uint32_t)level > t) ? 1u : 0u;
      wf[i] = (uint16_t)(wf[i] << 1) | pixel;
      peak[i] = 0;
    }
    line++;
  }

  // rows 2 and 3: unpack the column words into two pages
  static uint8_t img[256];
  for (uint32_t i=0;i<128;i++)
  {
    img[i]     = (uint8_t)(wf[i] & 0xffu);   // page 2: pixel rows 0..7
    img[i+128] = (uint8_t)(wf[i] >> 8);      // page 3: pixel rows 8..15
  }
  oled.bitmap(0, 2, 128, 4, img);
}

// draw whichever spectrum the current layout calls for
static void show_spectrum(const uint32_t signal)
{
  switch (spectrum_layout())
  {
    case LAYOUT_SPECTRUM:  show_spectrum_16(signal); break;
    case LAYOUT_CW:        show_spectrum_cw(signal); break;
    case LAYOUT_WATERFALL: show_waterfall(signal);   break;
    case LAYOUT_METER:     break; // show_meter_dial() draws
  }
}

static void show_frequency(void)
{
  // always exactly 10 characters so nothing after it moves:
  //   "14.222.000"   " 7.100.000"   "   936.000"
  const unsigned long f = radio.frequency;
  const unsigned long mhz = f / 1000000ul;
  const unsigned long khz = (f / 1000ul) % 1000ul;
  const unsigned long hz = f % 1000ul;
  char sz_frequency[16] = "";
  if (mhz>0ul)
  {
    snprintf(sz_frequency,sizeof(sz_frequency),"%2lu.%03lu.%03lu",mhz,khz,hz);
  }
  else
  {
    snprintf(sz_frequency,sizeof(sz_frequency),"   %3lu.%03lu",khz,hz);
  }
  oled.setFont(FONT6X8);
  text_at(COL_FREQ,0);
  oled.print(sz_frequency);
}

static void show_tuning_step(void)
{
  // right-aligned in 6 characters (10 .. 100000), ends in the last column
  char sz_step[12] = "";
  snprintf(sz_step,sizeof(sz_step),"%6lu",(unsigned long)radio.step);
  oled.setFont(FONT6X8);
  text_at(COL_STEP,0);
  oled.print(sz_step);
}

static void show_mode(void)
{
  const char *sz_mode = "XXX";
  switch (radio.mode)
  {
    case MODE_LSB: sz_mode = "LSB"; break;
    case MODE_USB: sz_mode = "USB"; break;
    case MODE_CWL: sz_mode = "CWL"; break;
    case MODE_CWU: sz_mode = "CWU"; break;
    case MODE_AM:  sz_mode = "AM";  break;
  }
  oled.setFont(FONT6X8);
  text_at(COL_MODE,0);
  oled.print(sz_mode);
}

static void show_band(void)
{
  const char *sz_band = "";
  switch (radio.band)
  {
    case BAND_80M: sz_band = "80M"; break;
    case BAND_40M: sz_band = "40M"; break;
    case BAND_30M: sz_band = "30M"; break;
    case BAND_20M: sz_band = "20M"; break;
    case BAND_17M: sz_band = "17M"; break;
    case BAND_15M: sz_band = "15M"; break;
    case BAND_12M: sz_band = "12M"; break;
    case BAND_10M: sz_band = "10M"; break;
    case BAND_SWL: sz_band = "SWL"; break;
  }
  oled.setFont(FONT6X8);
  text_at(COL_BAND,1);
  oled.print(sz_band);
}

static void show_bandwidth(void)
{
  const char *sz_bw = "";
  switch (radio.bandwidth)
  {
    case 1: sz_bw = "2000"; break;
    case 2: sz_bw = "2200"; break;
    case 3: sz_bw = "2400"; break;
    case 4: sz_bw = "2600"; break;
    case 5: sz_bw = "2800"; break;
  }
  oled.setFont(FONT6X8);
  text_at(COL_BW,1);
  oled.print(sz_bw);
}

static void show_nb(void)
{
  // NB and limiter
  if (radio.limiter == LIMITER_OFF)
  {
    char sz_nb[4] = "NB0";
    sz_nb[2] = (char)('0' + radio.nblevel);
    oled.setFont(FONT6X8);
    text_at(COL_NB,1);
    oled.print(sz_nb);
    return;
  }
  if (radio.nblevel == NB_OFF)
  {
    char sz_nl[4] = "NL0";
    sz_nl[2] = (char)('0' + radio.limiter);
    oled.setFont(FONT6X8);
    text_at(COL_NB,1);
    oled.print(sz_nl);
    return;
  }
  char sz_nbl[4] = "N00";
  sz_nbl[1] = (char)('0' + radio.nblevel);
  sz_nbl[2] = (char)('0' + radio.limiter);
  oled.setFont(FONT6X8);
  text_at(COL_NB,1);
  oled.print(sz_nbl);
}

static void show_cpu_usage(void)
{
  static uint32_t cpu_usage = 0;
  static uint32_t next_update = 0;
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update = now + 1000ul;
    const uint32_t usage = cpu.usage;
    const uint32_t count = cpu.count;
    cpu.usage = 0;
    cpu.count = 0;
    cpu_usage = count ? (100u * usage / count) >> 5 : 0u;
  }
  // 3 characters, so it must stay below 100%
  char sz_cpu[12] = "";
  snprintf(sz_cpu,sizeof(sz_cpu),"%2lu%%",(unsigned long)(cpu_usage>99u?99u:cpu_usage));
  oled.setFont(FONT6X8);
  text_at(COL_CPU,1);
  oled.print(sz_cpu);
}

static void show_notch(void)
{
  if (radio.notch_enable)
  {
    oled.setFont(FONT6X8);
    text_at(COL_NOTCH,1);
    oled.print("T");
  }
}

static void show_jnr(void)
{
  char sz_jnr[3] = "J0";
  sz_jnr[1] = (char)('0' + radio.jnrlevel);
  oled.setFont(FONT6X8);
  text_at(COL_JNR,1);
  oled.print(sz_jnr);
}

static void show_debug_value(const int32_t v1,const int32_t v2 = 0)
{
  if (v1 != 0)
  {
    oled.setFont(FONT6X8);
    text_at(0,3);
    oled.print(v1);
  }
  if (v2 != 0)
  {
    oled.setFont(FONT6X8);
    text_at(10,3);
    oled.print(v2);
  }
}

static void show_meter_dial(const uint32_t signal)
{
  // only in the meter layout; in the others the S-meter is the top line
  // of the spectrum
  if (spectrum_layout()!=LAYOUT_METER)
  {
    return;
  }
  static uint8_t meter[128] = {0};
  memset(meter,0,sizeof(meter));
  const uint32_t bar = min(signal,127);
  oled.setFont(FONT6X8);
  text_at(0,2);
  oled.print("1  3  5  7  9 +20 +40");
  for (uint32_t i=0;i<bar;i++)
  {
    meter[i] = 0b00111100;
  }
  oled.bitmap(0, 3, 128, 4, meter);
}

static const bool adjust_active(void)
{
  return set_spectrum_level || adj_spectrum_level || adj_notch_filter;
}

// while adjusting, this text replaces the whole of row 1
// (update_display() skips the normal row-1 fields)
static void show_adj_settings(void)
{
  char sz_adj[24] = "";
  if (set_spectrum_level || adj_spectrum_level)
  {
    // "Spectrum Level: -24dB" is exactly 21 characters
    snprintf(sz_adj,sizeof(sz_adj),"Spectrum Level: %ddB",(int)radio.level[radio.band]*6);
  }
  else if (adj_notch_filter)
  {
    snprintf(sz_adj,sizeof(sz_adj),"Notch: %luHz",(unsigned long)radio.notch);
  }
  oled.setFont(FONT6X8);
  text_at(0,1);
  oled.print(sz_adj);
}

static void show_cw_decode(void)
{
  // keep the last copy: if the mutex is busy this frame, the previous
  // text is redrawn rather than the row flashing blank
  static char z_buf[32] = "";
  if (!cw_decode_active())
  {
    z_buf[0] = '\0';
    return;
  }
  if (mutex_try_enter(&cw_decode_mutex,0ul))
  {
    strcpy(z_buf,(char*)cw_decode_buf);
    mutex_exit(&cw_decode_mutex);
  }
  oled.setFont(FONT6X8);
  text_at(0,2);
  oled.print(z_buf);
}

enum menu_button_state_t {MENU_BUTTON_IDLE,MENU_BUTTON_PRESSED,MENU_WAIT_RELEASE_0,MENU_WAIT_RELEASE_1};
enum menu_button_action_t {MENU_NONE,MENU_TOP_SELECT,MENU_OPTION_SELECT};
static menu_button_state_t menu_button_state = MENU_BUTTON_IDLE;
static menu_button_action_t menu_button_action = MENU_NONE;
static uint8_t menu_window = 0;
static uint8_t menu_current = 0;
static uint8_t option_window = 0;
static uint8_t option_current = 0;

static void show_menu(void)
{
  if (!radio.menu_active)
  {
    return;
  }
  oled.setFont(FONT6X8);
  if (menu_button_action==MENU_NONE) // MENU_MENU_SELECT
  {
    // display top level menu
    for (uint8_t i=menu_window,j=0;i<NUM_MENU_ITEMS && j<4;i++,j++)
    {
      char sz_menu_name[16] = "";
      memset(sz_menu_name,' ',sizeof(sz_menu_name));
      sz_menu_name[13] = '\0';
      for (int k=0;k<13;k++)
      {
        const char c = menu_options[i].menu_name[k];
        if (c=='\0') break;
        sz_menu_name[k] = c;
      }
      oled.setCursor(POS_MENU_X,j);
      if (i==menu_current) oled.invertOutput(true);
      oled.print(sz_menu_name);
      if (i==menu_current) oled.invertOutput(false);
    }
  }
  else if (menu_button_action==MENU_TOP_SELECT) // MENU_OPTION_SELECT
  {
    if (menu_current<NUM_MENU_ITEMS)
    {
      const uint8_t num_options = menu_options[menu_current].num_options;
      if (option_window<num_options && option_current<num_options)
      {
        oled.setFont(FONT6X8);
        oled.setCursor(POS_MENU_X,0);
        oled.print(menu_options[menu_current].menu_name);
        for (uint8_t i=option_window,j=1;i<num_options && j<4;i++,j++)
        {
          char sz_option_name[16] = "";
          memset(sz_option_name,' ',sizeof(sz_option_name));
          sz_option_name[12] = '\0';
          for (int k=1;k<12;k++)
          {
            const char c = menu_options[menu_current].options[i].option_name[k-1];
            if (c=='\0') break;
            sz_option_name[k] = c;
          }
          oled.setCursor(POS_MENU_X,j);
          if (i==option_current) oled.invertOutput(true);
          oled.print(sz_option_name);
          if (i==option_current) oled.invertOutput(false);
        }
      }
    }
  }
}

static char sz_popup[64] = "";
static char sz_popup_heading[64] = "";
static uint32_t popup_timeout = 0;

static void set_popup(const char *sz_message,const char *sz_heading = NULL)
{
  if (sz_message)
  {
    memset(sz_popup,0,sizeof(sz_popup));
    memset(sz_popup_heading,0,sizeof(sz_popup_heading));
    strcpy(sz_popup," ");
    strncat(sz_popup,sz_message,16);
    strcat(sz_popup,"          ");
    sz_popup[15] = '\0';
    if (sz_heading)
    {
      strcpy(sz_popup_heading,"  ");
      strncat(sz_popup_heading,sz_heading,12);
      strcat(sz_popup_heading,"          ");
      sz_popup_heading[15] = '\0';
    }
    else
    {
      strcpy(sz_popup_heading,"  ATTENTION:   ");
    }
    popup_timeout = millis() + POPUP_TIMEOUT;
  }
}

static void set_popup(volatile char *sz_message,const char *sz_heading = NULL)
{
  set_popup((const char*)sz_message,sz_heading);
}

static void show_popup(void)
{
  if (sz_popup[0] == '\0') return;
  if (millis() > popup_timeout)
  {
    sz_popup[0] = '\0';
    return;
  }
  // centred, so long messages ("Notch: SSB only") don't run off the edge
  oled.setFont(FONT6X8);
  oled.invertOutput(true);
  text_centre(sz_popup_heading,1);
  text_centre(sz_popup,2);
  oled.invertOutput(false);
}

static void update_display(const uint32_t signal_level = 0u)
{
  // layout: see the table above text_at()
  oled.clear();

  if (radio.menu_active)
  {
    // the menu owns the whole screen; otherwise the parts of the
    // normal display to the right of the menu text show through
    show_menu();
    show_popup();
    oled.switchFrame();
    return;
  }

  // row 0
  show_mode();
  show_frequency();
  show_tuning_step();

  // row 1: adjust text, else the status line, except in the waterfall
  // layout where show_spectrum() puts the 8-pixel spectrum here
  if (adjust_active())
  {
    show_adj_settings();
  }
  else if (spectrum_layout()!=LAYOUT_WATERFALL)
  {
    show_band();
    show_bandwidth();
    show_nb();
    show_jnr();
    show_notch();
    show_cpu_usage();
  }

  // rows 2 and 3 (later calls overwrite earlier ones), layout per
  // spectrum_layout(): 16-px spectrum / CW text + 8-px spectrum /
  // waterfall (plus 8-px spectrum on row 1) / S-meter dial
  show_spectrum(signal_level);
  show_meter_dial(signal_level);
  show_cw_decode();
  show_debug_value(debug_value_1,debug_value_2);
  show_popup();
  oled.switchFrame();
}

static void init_menu(void)
{
  menu_window = 0;
  menu_current = 0;
  option_window = 0;
  option_current = 0;
  menu_button_state = MENU_WAIT_RELEASE_0;
  menu_button_action = MENU_NONE;
}

static const option_value_t process_menu(void)
{
  static uint32_t menu_timeout = millis()+MENU_TIMEOUT;
  option_value_t option = OPTION_NONE;
  switch (menu_button_state)
  {
    case MENU_WAIT_RELEASE_0:
    {
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        menu_button_state = MENU_BUTTON_IDLE;
        delay(50);
      }
      menu_timeout = millis()+MENU_TIMEOUT;
      break;
    }
    case MENU_BUTTON_IDLE:
    {
      // has the button been pressed?
      if (digitalRead(PIN_ENCBUT)==LOW)
      {
        menu_button_state = MENU_WAIT_RELEASE_1;
        menu_timeout = millis()+MENU_TIMEOUT;
        delay(50);
      }
      break;
    }
    case MENU_WAIT_RELEASE_1:
    {
      if (digitalRead(PIN_ENCBUT)==HIGH)
      {
        menu_button_state = MENU_BUTTON_IDLE;
        switch (menu_button_action)
        {
          case MENU_NONE:          menu_button_action = MENU_TOP_SELECT;    break;
          case MENU_TOP_SELECT:    menu_button_action = MENU_OPTION_SELECT; break;
          case MENU_OPTION_SELECT: menu_button_action = MENU_NONE;          break;
        }
        delay(50);
      }
      menu_timeout = millis()+MENU_TIMEOUT;
      break;
    }
  }
  // process the button action
  switch (menu_button_action)
  {
    case MENU_NONE:
    {
      mutex_enter_blocking(&rotary_mutex);
      const int32_t rotary_delta = radio.tune;
      radio.tune = 0;
      mutex_exit(&rotary_mutex);
      if (rotary_delta>0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (menu_current<NUM_MENU_ITEMS-1)
        {
          menu_current++;
          if (menu_current>menu_window+3)
          {
            if (menu_window<NUM_MENU_ITEMS-1)
            {
              menu_window++;
            }
          }
        }
      }
      else if (rotary_delta<0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (menu_current>0)
        {
          menu_current--;
          if (menu_current<menu_window)
          {
            if (menu_window>0)
            {
              menu_window--;
            }          
          }
        }
      }
      break;
    }
    case MENU_TOP_SELECT:
    {
      if (menu_options[menu_current].menu_value==MENU_EXIT)
      {
        return OPTION_EXIT;
      }
      // process menu options
      const uint8_t num_options = menu_options[menu_current].num_options;
      mutex_enter_blocking(&rotary_mutex);
      const int32_t rotary_delta = radio.tune;
      radio.tune = 0;
      mutex_exit(&rotary_mutex);
      if (rotary_delta>0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (option_current<num_options-1)
        {
          option_current++;
          if (option_current>option_window+2)
          {
            if (option_window<num_options-1)
            {
              option_window++;
            }
          }
        }
      }
      else if (rotary_delta<0)
      {
        menu_timeout = millis()+MENU_TIMEOUT;
        if (option_current>0)
        {
          option_current--;
          if (option_current<option_window)
          {
            if (option_window>0)
            {
              option_window--;
            }          
          }
        }
      }
      break;
    }
    case MENU_OPTION_SELECT:
    {
      option = menu_options[menu_current].options[option_current].option_value;
      radio.menu_active = false;
      break;
    }
  }
  if (millis()>menu_timeout)
  {
    radio.menu_active = false;
  }
  return option;
}

void __not_in_flash_func(loop1)(void)
{
  // run DSP on core 1
  volatile static int32_t dac_h = 0;
  volatile static int32_t dac_l = 0;
  // wait for sample from FPGA
  if (digitalRead(PIN_DR)==LOW)
  {
    return;
  }
  volatile const uint32_t cpu_start = time_us_32();
  pwm_set_both_levels(audio_pwm,dac_h,dac_l);
  const uint32_t rx_sample = iqSPI.doTransfer(radio.tuning_word);
  const int16_t ii = (int16_t)(rx_sample>>16);
  const int16_t qq = (int16_t)(rx_sample & 0xffff);
#ifdef DEBUG_DDC_SAMPLE
  static uint32_t debug_count = 0;
  static uint32_t debug_abs_1 = 0;
  static uint32_t debug_abs_2 = 0;
  if (abs(ii) > debug_abs_1) debug_abs_1 = abs(ii);
  if (abs(qq) > debug_abs_2) debug_abs_2 = abs(qq);
  if (debug_count >= 15000)
  {
    debug_value_1 = debug_abs_1;
    debug_value_2 = debug_abs_2;
    debug_abs_1 = 0;
    debug_abs_2 = 0;
    debug_count = 0;
  }
  debug_count++;
#endif
#ifdef DEBUG_AGC_SAMPLE
  static uint32_t debug_count = 0;
  static uint32_t debug_abs_1 = 0;
  static uint32_t debug_abs_2 = 0;
  const int32_t agc = DSP::get_agc();
  if (abs(ii) > debug_abs_1) debug_abs_1 = abs(ii);
  if (abs(agc) > debug_abs_2) debug_abs_2 = abs(agc);
  if (debug_count >= 15000)
  {
    debug_value_1 = debug_abs_1;
    debug_value_2 = debug_abs_2;
    debug_abs_1 = 0;
    debug_abs_2 = 0;
    debug_count = 0;
  }
  debug_count++;
#endif
  adc_data_i[iq_sample_p] = ii;
  adc_data_q[iq_sample_p] = qq;
  iq_sample_p = (iq_sample_p+1) & (MAX_IQ_SAMPLES-1);
  int32_t dac_audio = 0;
  float cwsig = 0.0f;
  const uint8_t jnr = radio.jnrlevel;
  const uint8_t nb = radio.nblevel;
  const uint8_t lm = radio.limiter;
  const uint8_t bw = radio.bandwidth - 1;
  switch (radio.mode)
  {
    case MODE_LSB: dac_audio = DSP::process_ssb(qq,ii,jnr,bw,nb,lm); break;
    case MODE_USB: dac_audio = DSP::process_ssb(ii,qq,jnr,bw,nb,lm); break;
    case MODE_AM:  dac_audio = DSP::process_am(ii,qq,jnr);           break;
    case MODE_CWL: dac_audio = DSP::process_cw(qq,ii,cwsig);         break;
    case MODE_CWU: dac_audio = DSP::process_cw(ii,qq,cwsig);         break;
  }
  dac_audio = radio.mute?0:constrain(dac_audio,-2048l,+2047l);
  dac_audio += 2048l;
  dac_h = dac_audio >> 6;
  dac_l = dac_audio & 0x3f;
  if (radio.cwdecode != 0)
  {
    if (radio.mode==MODE_CWL || radio.mode==MODE_CWU)
    {
      static bool last_key_down = false;
      const uint8_t raw = radio.cwdecode==1?
        CWDECODE1::morse_decode(cwsig):
        CWDECODE2::morse_decode(cwsig);
      const bool cw_key_down = raw&0x80?true:false;
      const char raw_char = raw & 0x7f;
      if (cw_key_down != last_key_down || raw_char!='\0')
      {
        last_key_down = cw_key_down;
        mutex_enter_blocking(&cw_decode_mutex);
        cw_decode_buf[20] = '\0';
        cw_decode_buf[19] = cw_key_down?'*':' ';
        if (raw_char!='\0')
        {
          for (uint8_t i=0;i<18;i++)
          {
            const char c = cw_decode_buf[i+1];
            cw_decode_buf[i] = (c=='\0'?' ':c);
          }
          cw_decode_buf[18] = raw_char;
        }
        mutex_exit(&cw_decode_mutex);
      }
    }
    else
    {
      CWDECODE1::md1_initialised = 0;
      CWDECODE2::md2_initialised = 0;
      memset((char*)cw_decode_buf,0,sizeof(cw_decode_buf));
    }
  }
  else
  {
    CWDECODE1::md1_initialised = 0;
    CWDECODE2::md2_initialised = 0;
    memset((char*)cw_decode_buf,0,sizeof(cw_decode_buf));
  }
  // process tune
  static int32_t rotary = 0l;
  switch (r.process())
  {
    case DIR_CW:  rotary++; break;
    case DIR_CCW: rotary--; break;
  }
  if (rotary!=0)
  {
    // don't hang around if we can't own the mutex immediately
    // rotary will record any rotations
    if (mutex_try_enter(&rotary_mutex,0ul))
    {
      radio.tune += rotary;
      rotary = 0;
      mutex_exit(&rotary_mutex);
    }
  }
  volatile const uint32_t cpu_end = time_us_32();
  cpu.usage += (cpu_end-cpu_start);
  cpu.count++;
  while (digitalRead(PIN_DR)==HIGH)
  {
    // make sure data ready is LOW before returning
    tight_loop_contents();
  }
}

static void process_spectrum(void)
{
  uint32_t offset = 0;
  const uint32_t sample_p = iq_sample_p;
  if (sample_p<800)
  {
    // first half is in use, get the second half
    offset = SPECTRUM_BUFFER;
  }
  else if (sample_p>1023 && sample_p<1800)
  {
    // second half is in use, get the first half
    offset = 0;
  }
  else
  {
    return;
  }
  static int16_t data_re[SPECTRUM_BUFFER] = {0};
  static int16_t data_im[SPECTRUM_BUFFER] = {0};
  const int8_t gain = radio.level[radio.band];
  for (uint32_t i=0;i<SPECTRUM_BUFFER;i++)
  {
    data_re[i] = adc_data_i[i+offset];
    data_im[i] = adc_data_q[i+offset];
  }
  spectrum::process(data_re,data_im,magnitude,gain);
}

static const radio_mode_t get_mode_auto(void)
{
  if (!radio.mode_auto)
  {
    return radio.mode;
  }
  if (radio.band==BAND_SWL)
  {
    return MODE_AM;
  }
  if (radio.frequency<10000000ul)
  {
    if (radio.frequency==7074000ul)
    {
      return MODE_USB;
    }
    if (radio.frequency==3573000ul)
    {
      return MODE_USB;
    }
    if (radio.frequency<7060000ul && radio.frequency>=7000000ul)
    {
      return MODE_CWL;
    }
    return MODE_LSB;
  }
  else if (radio.frequency<14060000ul && radio.frequency>=14000000ul)
  {
    return MODE_CWU;
  }
  return MODE_USB;
}

static void set_notch_filter(void)
{
  // rotary sets notch frequency
  static uint32_t old_notch_frequency = radio.notch;
  mutex_enter_blocking(&rotary_mutex);
  const int32_t notch_delta = radio.tune;
  radio.tune = 0;
  mutex_exit(&rotary_mutex);
  volatile int32_t f = (int32_t)radio.notch + notch_delta * 10; // 10 Hz per detent
  f = constrain(f,NOTCH_LOW,NOTCH_HIGH);
  radio.notch = f;
  if (old_notch_frequency != f)
  {
    old_notch_frequency = f;
    DSP::notch.set((float)f, 100.0f, 30.0f);
  }
  if (digitalRead(PIN_ENCBUT)==LOW)
  {
    // if button pressed, exit
    delay(50);
    while (digitalRead(PIN_ENCBUT)==LOW)
    {
      tight_loop_contents();
    }
    delay(50);
    adj_notch_filter = false;
  }
}

static void about_update(void)
{
  radio.update = BOOTLOADER_REQUEST;
  save_settings();
  set_popup("Connect USB","UNPLUG PWR");
}

static void about_reboot(void)
{
  constexpr uint32_t size = 2;
  static const char *s = "Rebooting...";
  oled.clear();
  oled.setFont(FONT6X8);
  text_centre(s,1);
  oled.switchFrame();
  watchdog_reboot(0, 0, 1000);
  for (;;)
  {
    tight_loop_contents();
  }
}

/*
 * general UI processing
 */
void loop(void)
{
  // run UI on core 0
  static uint32_t old_frequency = radio.frequency;
  static uint32_t old_band = radio.band;
  static uint32_t old_jnrlevel = radio.jnrlevel;
  static uint32_t old_bandwidth = radio.bandwidth;
  static int8_t old_level = 0;
  static bool old_notch_enable = radio.notch_enable;
  static mode_t old_mode = radio.mode;

  // process button press
  // short press: change step
  // long press: menu options
  enum button_state_t {BUTTON_IDLE,BUTTON_TEST_SHORT,BUTTON_WAIT_RELEASE};
  enum button_action_t {BUTTON_NO_PRESS,BUTTON_SHORT_PRESS,BUTTON_LONG_PRESS};
  static button_state_t button_state = BUTTON_IDLE;
  static uint32_t button_timer = 0;
  button_action_t button_action = BUTTON_NO_PRESS;
  if (radio.menu_active)
  {
    const option_value_t option = process_menu();
    switch (option)
    {
      case OPTION_MODE_LSB:        radio.mode = MODE_LSB; radio.mode_auto = false; break;
      case OPTION_MODE_USB:        radio.mode = MODE_USB; radio.mode_auto = false; break;
      case OPTION_MODE_CWL:        radio.mode = MODE_CWL; radio.mode_auto = false; break;
      case OPTION_MODE_CWU:        radio.mode = MODE_CWU; radio.mode_auto = false; break;
      case OPTION_MODE_AM:         radio.mode = MODE_AM;  radio.mode_auto = false; break;
      case OPTION_MODE_AUTO:       radio.mode_auto = true;                         break;
      case OPTION_STEP_10:         radio.step = 10U;                               break;
      case OPTION_STEP_100:        radio.step = 100U;                              break;
      case OPTION_STEP_500:        radio.step = 500U;                              break;
      case OPTION_STEP_1000:       radio.step = 1000U;                             break;
      case OPTION_STEP_5000:       radio.step = 5000U;                             break;
      case OPTION_STEP_10000:      radio.step = 10000U;                            break;
      case OPTION_STEP_100000:     radio.step = 100000U;                           break;
      case OPTION_BAND_80M:        radio.band = BAND_80M;                          break;
      case OPTION_BAND_40M:        radio.band = BAND_40M;                          break;
      case OPTION_BAND_30M:        radio.band = BAND_30M;                          break;
      case OPTION_BAND_20M:        radio.band = BAND_20M;                          break;
      case OPTION_BAND_17M:        radio.band = BAND_17M;                          break;
      case OPTION_BAND_15M:        radio.band = BAND_15M;                          break;
      case OPTION_BAND_12M:        radio.band = BAND_12M;                          break;
      case OPTION_BAND_10M:        radio.band = BAND_10M;                          break;
      case OPTION_BAND_SWL:        radio.band = BAND_SWL;                          break;
      case OPTION_BW_2000:         radio.bandwidth = 1ul;                          break;
      case OPTION_BW_2200:         radio.bandwidth = 2ul;                          break;
      case OPTION_BW_2400:         radio.bandwidth = 3ul;                          break;
      case OPTION_BW_2600:         radio.bandwidth = 4ul;                          break;
      case OPTION_BW_2800:         radio.bandwidth = 5ul;                          break;
      case OPTION_NOTCH_ON:        radio.notch_enable = true;                      break;
      case OPTION_NOTCH_OFF:       radio.notch_enable = false;                     break;
      case OPTION_DECODE_ADAPTIVE: radio.cwdecode = 1;                             break;
      case OPTION_DECODE_SCHMITT:  radio.cwdecode = 2;                             break;
      case OPTION_CWDECODE_OFF:    radio.cwdecode = 0;                             break;
      case OPTION_SPEC_SETLEVEL:   set_spectrum_level = true;                      break;
      case OPTION_SPEC_ADJLEVEL:   adj_spectrum_level = true;                      break;
      case OPTION_SPECTRUM_ON:     radio.spectrum = true;                          break;
      case OPTION_SPECTRUM_OFF:    radio.spectrum = false;                         break;
      case OPTION_WATERFALL_ON:    radio.waterfall = true;                         break;
      case OPTION_WATERFALL_OFF:   radio.waterfall = false;                        break;
      case OPTION_WATER_SLOW:      radio.waterspeed = 0;                           break;
      case OPTION_WATER_MEDIUM:    radio.waterspeed = 1;                           break;
      case OPTION_WATER_FAST:      radio.waterspeed = 2;                           break;
      case OPTION_JNR_LEVEL1:      radio.jnrlevel = JNR_LEVEL1;                    break;
      case OPTION_JNR_LEVEL2:      radio.jnrlevel = JNR_LEVEL2;                    break;
      case OPTION_JNR_LEVEL3:      radio.jnrlevel = JNR_LEVEL3;                    break;
      case OPTION_JNR_OFF:         radio.jnrlevel = JNR_OFF;                       break;
      case OPTION_NB_LEVEL1:       radio.nblevel = NB_LEVEL1;                      break;
      case OPTION_NB_LEVEL2:       radio.nblevel = NB_LEVEL2;                      break;
      case OPTION_NB_LEVEL3:       radio.nblevel = NB_LEVEL3;                      break;
      case OPTION_NB_LEVEL4:       radio.nblevel = NB_LEVEL4;                      break;
      case OPTION_NB_LEVEL5:       radio.nblevel = NB_LEVEL5;                      break;
      case OPTION_NB_OFF:          radio.nblevel = NB_OFF;                         break;
      case OPTION_LIMITER_LEVEL1:  radio.limiter = LIMITER_LEVEL1;                 break;
      case OPTION_LIMITER_LEVEL2:  radio.limiter = LIMITER_LEVEL2;                 break;
      case OPTION_LIMITER_LEVEL3:  radio.limiter = LIMITER_LEVEL3;                 break;
      case OPTION_LIMITER_LEVEL4:  radio.limiter = LIMITER_LEVEL4;                 break;
      case OPTION_LIMITER_LEVEL5:  radio.limiter = LIMITER_LEVEL5;                 break;
      case OPTION_LIMITER_OFF:     radio.limiter = LIMITER_OFF;                    break;
      case OPTION_VERSION:         set_popup(sz_version,"VERSION:");               break;
      case OPTION_REBOOT:          about_reboot();                                 break;
      case OPTION_UPDATE:          about_update();                                 break;
      case OPTION_EXIT:            radio.menu_active = false;                      break;
    }

    if (set_spectrum_level)
    {
      // restore the saved level
      radio.level[radio.band] = radio.slevel[radio.band];
      old_level = radio.level[radio.band];
    }
    if (adj_spectrum_level)
    {
      // only changing current level, not saving it
      old_level = radio.level[radio.band];
    }

    // notch enable/disable
    if (old_notch_enable != radio.notch_enable)
    {
      if (radio.mode==MODE_LSB || radio.mode==MODE_USB)
      {
        old_notch_enable = radio.notch_enable;
        adj_notch_filter = radio.notch_enable;
        DSP::notch.enable(radio.notch_enable);
      }
      else
      {
        // maintain notch condition
        radio.notch_enable = old_notch_enable;
        set_popup("SSB only");
      }
    }

    // when settings change save them
    bool settings_changed = false;

    // JNR level
    if (radio.jnrlevel != old_jnrlevel)
    {
      old_jnrlevel = radio.jnrlevel;
      settings_changed = true;
    }

    // bandwidth
    if (radio.bandwidth != old_bandwidth)
    {
      old_bandwidth = radio.bandwidth;
      settings_changed = true;
    }

    // save the settings
    if (settings_changed)
    {
      save_settings();
    }

    // mode may change in auto
    const radio_mode_t auto_mode = get_mode_auto();
    if (radio.mode != auto_mode)
    {
      radio.mode = auto_mode;
    }

    // update frequency if mode changes
    // delay enabling Mic for Vox
    if (radio.mode!=old_mode)
    {
      old_mode = radio.mode;
      set_frequency();
    }

    // set band
    if (radio.band!=old_band)
    {
      // on this radio, changing the band
      // is just changing the frequency
      save_data[old_band].frequency = radio.frequency;
      radio.frequency = save_data[radio.band].frequency;
      old_frequency = radio.frequency;
      old_band = radio.band;
      // mute during change
      set_frequency();
    }
  }
  else
  {
    // menu not active
    if (set_spectrum_level || adj_spectrum_level || adj_notch_filter)
    {
      if (adj_notch_filter)
      {
        set_notch_filter();
      }
      else
      {
        // rotary sets spectrum level
        mutex_enter_blocking(&rotary_mutex);
        const int32_t level_delta = radio.tune;
        radio.tune = 0;
        mutex_exit(&rotary_mutex);
        int8_t new_level = radio.level[radio.band] + level_delta;
        new_level = constrain(new_level,SPECTRUM_LEVEL_MIN,SPECTRUM_LEVEL_MAX);
        radio.level[radio.band] = new_level;
        if (digitalRead(PIN_ENCBUT)==LOW)
        {
          // if button pressed
          // save new level (if changed)
          // wait for button release
          if (set_spectrum_level && new_level!=old_level)
          {
            // remember saved level
            radio.slevel[radio.band] = new_level;
            save_settings();
          }
          delay(50);
          while (digitalRead(PIN_ENCBUT)==LOW)
          {
            tight_loop_contents();
          }
          delay(50);
          set_spectrum_level = false;
          adj_spectrum_level = false;
        }
      }
    }
    else
    {
      switch (button_state)
      {
        case BUTTON_IDLE:
        {
          if (digitalRead(PIN_ENCBUT)==LOW)
          {
            button_state = BUTTON_TEST_SHORT;
            button_timer = millis()+BUTTON_LONG_PRESS_TIME;
            delay(50);
          }
          break;
        }
        case BUTTON_TEST_SHORT:
        {
          const uint32_t now = millis();
          if (digitalRead(PIN_ENCBUT)==HIGH)
          {
            button_state = BUTTON_IDLE;
            if (now<button_timer)
            {
              button_action = BUTTON_SHORT_PRESS;
            }
            delay(50);
            break;
          }
          if (now>button_timer)
          {
            button_state = BUTTON_WAIT_RELEASE;
            button_action = BUTTON_LONG_PRESS;
          }
          break;
        }
        case BUTTON_WAIT_RELEASE:
        {
          if (digitalRead(PIN_ENCBUT)==HIGH)
          {
            button_state = BUTTON_IDLE;
            delay(50);
          }
          break;
        }
      }

      // process button action
      switch (button_action)
      {
        case BUTTON_SHORT_PRESS:
        {
          radio.step *= 10u;
          if (radio.step>10000u) radio.step = 100u;
          break;
        }
        case BUTTON_LONG_PRESS:
        {
          init_menu();
          radio.menu_active = true;
          break;
        }
      }

      // set the mode based on frequency if auto
      const radio_mode_t auto_mode = get_mode_auto();
      if (radio.mode != auto_mode)
      {
        radio.mode = auto_mode;
      }

      // process main tuning
      mutex_enter_blocking(&rotary_mutex);
      const int32_t tuning_delta = radio.tune;
      radio.tune = 0;
      mutex_exit(&rotary_mutex);
      volatile uint32_t new_frequency = radio.frequency;
      if (tuning_delta != 0)
      {
        const uint32_t step_modifer = (tuning_delta>0)?new_frequency:(new_frequency+radio.step-1);
        new_frequency = (step_modifer / radio.step) * radio.step;
        new_frequency += tuning_delta * radio.step;
        new_frequency = constrain(new_frequency,bands[radio.band].lo,bands[radio.band].hi);
      }
      if (new_frequency!=old_frequency || radio.mode!=old_mode)
      {
        radio.frequency = new_frequency;
        old_frequency = new_frequency;
        old_mode = radio.mode;
        set_frequency();
      } // frequency change
    } // spectrum level
  } // menu active

  // process spectrum
  static uint32_t process_spectrum_txrx = 0;
  if (millis()>process_spectrum_txrx)
  {
    process_spectrum();
  }

  // update the display every 50ms
  static uint32_t next_update = 0;
  const uint32_t now = millis();
  if (now>next_update)
  {
    next_update = now + 50ul;
#ifdef DEBUG_METER_DBM
    debug_value_1 = (int32_t)lroundf(DSP::meter_dbm);
#endif    
    update_display(DSP::smeter());
  }
}