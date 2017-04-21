#
# Component Makefile
#

COMPONENT_ADD_INCLUDEDIRS := drivers/include finsh bsp
COMPONENT_PRIV_INCLUDEDIRS := drivers/include finsh bsp
COMPONENT_SRCDIRS := drivers/src drivers/serial finsh bsp
COMPONENT_ADD_LDFLAGS += $(COMPONENT_BUILD_DIR)/finsh/cmd.o $(COMPONENT_BUILD_DIR)/finsh/msh_cmd.o
