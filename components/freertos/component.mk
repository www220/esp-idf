#
# Component Makefile
#

COMPONENT_ADD_LDFLAGS = -l$(COMPONENT_NAME) -Wl,--undefined=uxTopUsedPriority
COMPONENT_ADD_INCLUDEDIRS := include include/freertos include/rtthread
COMPONENT_PRIV_INCLUDEDIRS := include/freertos include/rtthread

#ifdef CONFIG_SYSVIEW_ENABLE
#COMPONENT_ADD_INCLUDEDIRS += app_trace
#endif
