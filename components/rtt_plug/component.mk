#
# Component Makefile
#

CFLAGS += -DLUA_OPTIMIZE_MEMORY=2

COMPONENT_ADD_INCLUDEDIRS := drivers/include finsh bsp lua
COMPONENT_PRIV_INCLUDEDIRS := drivers/include finsh bsp lua lua/inc
COMPONENT_SRCDIRS := drivers/src drivers/serial finsh bsp lua lua/modules
COMPONENT_ADD_LDFLAGS += $(COMPONENT_BUILD_DIR)/finsh/cmd.o $(COMPONENT_BUILD_DIR)/finsh/msh_cmd.o $(COMPONENT_BUILD_DIR)/lua/lua.o
