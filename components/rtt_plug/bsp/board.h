/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */

#ifndef __USER_CONFIG_H__
#define __USER_CONFIG_H__

#include "driver/periph_ctrl.h"
#include "driver/rtc_io.h"
#include "driver/gpio.h"

#define RT_USING_UART0
#define RT_USING_UART1
#define RT_USING_UART2

#define CONSOLE_DEVICE "uart0"
#define FINSH_DEVICE_NAME "telnet"
#define PPP_DEVICE_NAME "uart0"

void rt_hw_board_init(void);
int rt_hw_eth_init(void);
int rt_hw_telnet_init(void);
void rt_hw_putc_init(int type);
void rt_hw_console_putc(char c);

#define PZ_SKIPPRJ      0
#define PZ_WAIT         1
#define PZ_NTP          2
#define PZ_WIFILINK     3
#define PZ_ETHLINK      4
#define PZ_APLINK       5
#define PZ_PPPLINK      6
#define PZ_LICCHECK     7

extern volatile int sys_stauts;
extern volatile int uptime_count;
extern volatile unsigned char PZ[16];
extern char RTT_USER[16];
extern char RTT_PASS[36];
extern char RTT_NTP[32];
extern unsigned long long RTT_PRJNO;
extern unsigned int RTT_DNS;

#define rttIoScanDir "/spi/ioscan"
#define rttLogDir "/spi/log"
#define LUA_PATH_DEFAULT rttIoScanDir"/?.lc;"rttIoScanDir"/?.lua;"

char *GetPrivateStringData(const char *name, char *buf, int buflen, const char *file);
int SetPrivateStringData(const char *name, const char *buf, const char *file);

extern unsigned xthal_get_ccount(void);
inline uint32_t rt_hw_cpu_ms() { return xthal_get_ccount() / (CPU_CLK_FREQ_ROM / 1000); }
int sym_settimenow(int time);

#endif

