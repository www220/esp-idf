#
# Component Makefile
#

COMPONENT_ADD_LDFLAGS += -Wl,--undefined=uxTopUsedPriority
COMPONENT_ADD_INCLUDEDIRS := include include/rtthread
COMPONENT_PRIV_INCLUDEDIRS := include/freertos include/rtthread
