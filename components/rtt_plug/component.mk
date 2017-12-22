#
# Component Makefile
#

CFLAGS += -DLUA_OPTIMIZE_MEMORY=2

COMPONENT_ADD_INCLUDEDIRS := drivers/include finsh bsp lua lua/inc dfs
COMPONENT_PRIV_INCLUDEDIRS := drivers/include finsh bsp lua lua/inc dfs
COMPONENT_SRCDIRS := drivers/src drivers/serial finsh bsp lua lua/modules dfs
COMPONENT_ADD_LDFLAGS += $(COMPONENT_BUILD_DIR)/dfs/vi.o
COMPONENT_ADD_LDFLAGS += $(COMPONENT_BUILD_DIR)/bsp/tcpecho.o $(COMPONENT_BUILD_DIR)/bsp/netio.o $(COMPONENT_BUILD_DIR)/bsp/ntp.o $(COMPONENT_BUILD_DIR)/bsp/ping.o 
