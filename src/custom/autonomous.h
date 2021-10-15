#pragma once
#include "../../custom.h"

#ifdef CUSTOM_AUTONOMOUS
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "autonomous"
#endif
