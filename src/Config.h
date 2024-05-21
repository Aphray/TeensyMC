#pragma once

#if defined __has_include
#   if __has_include("TMC_userConfig.h")
#       include "TMC_userConfig.h"
#   else
#       include "TMC_defaultConfig.h"
#   endif
#endif