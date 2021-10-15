#pragma once

// select one custom variant below
#if false
  #define CUSTOM_AUTONOMOUS 1
  #include "src/custom/autonomous.h"
#endif

#if false
  #define CUSTOM_CW 1
  #include "src/custom/cw.h"
#endif


void setupCustom();
void loopCustom();
