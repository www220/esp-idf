/* RT-Thread config file */
#ifndef __RTTHREAD_CFG_H__
#define __RTTHREAD_CFG_H__

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "freertos/queue.h"
#include "freertos/StackMacros.h"

/* RT_NAME_MAX*/
#define RT_NAME_MAX	16

/* RT_ALIGN_SIZE*/
#define RT_ALIGN_SIZE	4

/* PRIORITY_MAX */
#define RT_THREAD_PRIORITY_MAX	32

/* Tick per Second */
#define RT_TICK_PER_SECOND	1000
#if RT_TICK_PER_SECOND == 1000
#define M2T(x) (x)
#else
#define M2T(x) (((x)*RT_TICK_PER_SECOND+999)/1000)
#endif

/* SECTION: RT_DEBUG */
/* Thread Debug */
#define RT_DEBUG

/* Using Dynamic Heap Management */
#define RT_USING_HEAP
#define xRT_USING_SEMAPHORE
#define xRT_USING_MUTEX

/* SECTION: Device System */
/* Using Device System */
#define RT_USING_DEVICE
#define RT_USING_DEVICE_IPC
#define RT_USING_SERIAL
#define RT_USING_SPI
#define RT_USING_I2C
#define RT_USING_I2C_BITOPS
#define RT_USING_PIN

/* SECTION: Console options */
#define RT_USING_CONSOLE
/* the buffer size of console*/
#define RT_CONSOLEBUF_SIZE	128

/* SECTION: finsh, a C-Express shell */
#define RT_USING_FINSH
#define FINSH_USING_MSH
#define FINSH_USING_MSH_DEFAULT
#define FINSH_USING_MSH_ONLY

/* Using symbol table */
#define FINSH_USING_SYMTAB
#define FINSH_USING_DESCRIPTION
#define FINSH_THREAD_STACK_SIZE	8192

/* SECTION: a runtime libc library */
/* a runtime libc library */
#define RT_USING_NEWLIB

/* SECTION: device filesystem */
/* Using Device file system */
#define RT_USING_DFS
#define DFS_USING_WORKDIR

#endif
