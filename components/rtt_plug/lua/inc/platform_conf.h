// Generic platform configuration file

#ifndef __PLATFORM_CONF_H__
#define __PLATFORM_CONF_H__

#include "auxmods.h"

#define LUA_PLATFORM_LIBS_REG \
  {LUA_LOADLIBNAME,	luaopen_package }

#define LUA_PLATFORM_LIBS_ROM \
  _ROM (LUA_STRLIBNAME, luaopen_string, strlib )\
  _ROM (LUA_MATHLIBNAME, luaopen_math, math_map )\
  _ROM (LUA_TABLIBNAME, luaopen_table, tab_funcs )\
  _ROM( AUXLIB_IOSCAN, luaopen_ioscan, ioscan_map )\
  _ROM( AUXLIB_DSERVE, luaopen_dserve, dserve_map )\
  _ROM( AUXLIB_PACK, luaopen_pack, pack_map )\
  _ROM( AUXLIB_BIT, luaopen_bit, bit_map )

#endif
