#ifndef MENU_H
#define MENU_H

/*

 Step
  10
  100
  500
  1000
  10000
  Exit

 Mode
   LSB
   USB
   CWL
   CWU
   Auto
   Exit

  Band
    80M
    40M
    ...
    Exit

  ...

  Exit
    Exit

*/

#define NUM_MENU_ITEMS 11U
#define NUM_MENU_OPTIONS 10U

enum menu_top_t
{
  MENU_BAND,
  MENU_STEP,
  MENU_MODE,
  MENU_BANDWIDTH,
  MENU_NOTCH,
  MENU_CW_DECODE,
  MENU_SPECTRUM,
  MENU_JNR,
  MENU_NB,
  MENU_ABOUT,
  MENU_EXIT
};

enum option_value_t
{
  OPTION_STEP_10,
  OPTION_STEP_100,
  OPTION_STEP_500,
  OPTION_STEP_1000,
  OPTION_STEP_5000,
  OPTION_STEP_10000,
  OPTION_STEP_100000,
  OPTION_MODE_USB,
  OPTION_MODE_LSB,
  OPTION_MODE_CWL,
  OPTION_MODE_CWU,
  OPTION_MODE_DGL,
  OPTION_MODE_DGU,
  OPTION_MODE_FT8,
  OPTION_MODE_AM,
  OPTION_MODE_AUTO,
  OPTION_BAND_80M,
  OPTION_BAND_40M,
  OPTION_BAND_30M,
  OPTION_BAND_20M,
  OPTION_BAND_17M,
  OPTION_BAND_15M,
  OPTION_BAND_12M,
  OPTION_BAND_10M,
  OPTION_BAND_SWL,
  OPTION_BW_2000,
  OPTION_BW_2200,
  OPTION_BW_2400,
  OPTION_BW_2600,
  OPTION_BW_2800,
  OPTION_NOTCH_ON,
  OPTION_NOTCH_OFF,
  OPTION_DECODE_ADAPTIVE,
  OPTION_DECODE_SCHMITT,
  OPTION_CWDECODE_OFF,
  OPTION_SPECTRUM_ON,
  OPTION_SPECTRUM_OFF,
  OPTION_WATERFALL_ON,
  OPTION_WATERFALL_OFF,
  OPTION_WATER_SLOW,
  OPTION_WATER_MEDIUM,
  OPTION_WATER_FAST,
  OPTION_SPEC_SETLEVEL,
  OPTION_SPEC_ADJLEVEL,
  OPTION_JNR_OFF,
  OPTION_JNR_LEVEL1,
  OPTION_JNR_LEVEL2,
  OPTION_JNR_LEVEL3,
  OPTION_NB_OFF,
  OPTION_NB_LEVEL1,
  OPTION_NB_LEVEL2,
  OPTION_NB_LEVEL3,
  OPTION_NB_LEVEL4,
  OPTION_NB_LEVEL5,
  OPTION_VERSION,
  OPTION_REBOOT,
  OPTION_UPDATE,
  OPTION_NONE,
  OPTION_EXIT
};

struct options_t
{
  option_value_t option_value;
  const char* option_name;
};

static const struct
{
  const menu_top_t menu_value;
  const char *menu_name;
  const uint8_t num_options;
  const options_t options[NUM_MENU_OPTIONS];
}
menu_options[NUM_MENU_ITEMS] =
{
  {
    MENU_BAND,
    "Band",
    10U,
    {
      {OPTION_BAND_80M,"80M"},
      {OPTION_BAND_40M,"40M"},
      {OPTION_BAND_30M,"30M"},
      {OPTION_BAND_20M,"20M"},
      {OPTION_BAND_17M,"17M"},
      {OPTION_BAND_15M,"15M"},
      {OPTION_BAND_12M,"12M"},
      {OPTION_BAND_10M,"10M"},
      {OPTION_BAND_SWL,"SWL"},
      {OPTION_EXIT,"Exit"}
    }
  },
  {
    MENU_MODE,
    "Mode",
    7U,
    {
      {OPTION_MODE_AUTO,"AUTO"},
      {OPTION_MODE_LSB,"LSB"},
      {OPTION_MODE_USB,"USB"},
      {OPTION_MODE_CWL,"CWL"},
      {OPTION_MODE_CWU,"CWU"},
      {OPTION_MODE_AM,"AM"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_STEP,
    "Step",
    8U,
    {
      {OPTION_STEP_10,"10"},
      {OPTION_STEP_100,"100"},
      {OPTION_STEP_500,"500"},
      {OPTION_STEP_1000,"1000"},
      {OPTION_STEP_5000,"5000"},
      {OPTION_STEP_10000,"10000"},
      {OPTION_STEP_100000,"100000"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_JNR,
    "JNR",
    5U,
    {
      {OPTION_JNR_LEVEL1,"Level 1"},
      {OPTION_JNR_LEVEL2,"Level 2"},
      {OPTION_JNR_LEVEL3,"Level 3"},
      {OPTION_JNR_OFF,"Off"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_NB,
    "Blanker",
    7U,
    {
      {OPTION_NB_LEVEL1,"Level 1"},
      {OPTION_NB_LEVEL2,"Level 2"},
      {OPTION_NB_LEVEL3,"Level 3"},
      {OPTION_NB_LEVEL4,"Level 4"},
      {OPTION_NB_LEVEL5,"Level 5"},
      {OPTION_NB_OFF,"Off"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_BANDWIDTH,
    "Bandwidth",
    6U,
    {
      {OPTION_BW_2000,"2000 Hz"},
      {OPTION_BW_2200,"2200 Hz"},
      {OPTION_BW_2400,"2400 Hz"},
      {OPTION_BW_2600,"2600 Hz"},
      {OPTION_BW_2800,"2800 Hz"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_NOTCH,
    "Notch Filter",
    3U,
    {
      {OPTION_NOTCH_ON,"Notch On"},
      {OPTION_NOTCH_OFF,"Notch Off"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_SPECTRUM,
    "Spectrum",
    10U,
    {
      {OPTION_SPECTRUM_ON,"On"},
      {OPTION_SPECTRUM_OFF,"Off"},
      {OPTION_SPEC_SETLEVEL,"Set Level"},
      {OPTION_SPEC_ADJLEVEL,"Adj Level"},
      {OPTION_WATERFALL_ON,"Water On"},
      {OPTION_WATERFALL_OFF,"Water Off"},
      {OPTION_WATER_SLOW,"Speed Slow"},
      {OPTION_WATER_MEDIUM,"Speed Med."},
      {OPTION_WATER_FAST,"Speed Fast"},
      {OPTION_EXIT,"Exit"}
    }
  },
  {
    MENU_CW_DECODE,
    "CW Decode",
    4U,
    {
      {OPTION_DECODE_ADAPTIVE,"Adaptive"},
      {OPTION_DECODE_SCHMITT,"Schmitt"},
      {OPTION_CWDECODE_OFF,"Off"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_ABOUT,
    "About",
    4U,
    {
      {OPTION_VERSION,"Version"},
      {OPTION_REBOOT,"Reboot"},
      {OPTION_UPDATE,"Update"},
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  },
  {
    MENU_EXIT,
    "Exit",
    1U,
    {
      {OPTION_EXIT,"Exit"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"},
      {OPTION_NONE,"None"}
    }
  }
};

#endif