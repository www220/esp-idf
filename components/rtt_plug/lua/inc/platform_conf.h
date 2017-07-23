// Generic platform configuration file

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "auxmods.h"

#define LUA_PLATFORM_LIBS_ROM \
  _ROM (LUA_STRLIBNAME, luaopen_string, strlib )\
  _ROM (LUA_MATHLIBNAME, luaopen_math, math_map )

#endif
