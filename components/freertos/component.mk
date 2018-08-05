#
# Component Makefile
#

COMPONENT_ADD_LDFLAGS += -Wl,--undefined=uxTopUsedPriority
COMPONENT_ADD_INCLUDEDIRS := include include/rtthread
COMPONENT_PRIV_INCLUDEDIRS := include/freertos include/rtthread

tasks.o event_groups.o timers.o queue.o ringbuf.o: CFLAGS += -D_ESP_FREERTOS_INTERNAL
