#pragma once

// select one custom variant below

#define CUSTOM_AUTONOMOUS
// #define CUSTOM_CW
// #define CUSTOM_CW_LETNANY
// #define CUSTOM_RESIN_MIXER
// #define CUSTOM_HEATBED
// #define CUSTOM_MFRC522_TEST


#ifdef CUSTOM_AUTONOMOUS
  #include "src/custom/autonomous.h"
#endif

#ifdef CUSTOM_CW
  #include "src/custom/cw.h"
#endif

#ifdef CUSTOM_CW_LETNANY
  #include "src/custom/cw_letnany.h"
#endif

#ifdef CUSTOM_RESIN_MIXER
  #include "src/custom/resin_mixer.h"
#endif

#ifdef CUSTOM_HEATBED
  #include "src/custom/heatbed.h"
#endif

#ifdef CUSTOM_MFRC522_TEST
  #include "src/custom/mfrc522_test.h"
#endif



#ifdef CUSTOM_SETUP
  void setupCustom();
#endif

#ifdef CUSTOM_LOOP
  void loopCustom();
#endif
