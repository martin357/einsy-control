#pragma once

// select one custom variant below

// #define CUSTOM_AUTONOMOUS
#define CUSTOM_CW


#ifdef CUSTOM_AUTONOMOUS
  #include "src/custom/autonomous.h"
#endif

#ifdef CUSTOM_CW
  #include "src/custom/cw.h"
#endif



#ifdef CUSTOM_SETUP
  void setupCustom();
#endif

#ifdef CUSTOM_LOOP
  void loopCustom();
#endif
