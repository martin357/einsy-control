#pragma once

// select one custom variant below

// #define CUSTOM_AUTONOMOUS
// #define CUSTOM_CW
#define CUSTOM_CW_LETNANY


#ifdef CUSTOM_AUTONOMOUS
  #include "src/custom/autonomous.h"
#endif

#ifdef CUSTOM_CW
  #include "src/custom/cw.h"
#endif

#ifdef CUSTOM_CW_LETNANY
  #include "src/custom/cw_letnany.h"
#endif



#ifdef CUSTOM_SETUP
  void setupCustom();
#endif

#ifdef CUSTOM_LOOP
  void loopCustom();
#endif
