#pragma once
#include "../../custom.h"

#ifdef CUSTOM_CW
  #ifdef CUSTOM
    #errot MULTIPLE CUSTOM VARIANTS ENABLED!
  #endif
  #define CUSTOM "cw"
#endif
