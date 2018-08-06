// Platform-dependent functions

#include "lua.h"
#include "platform.h"
#include "platform_conf.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <signal.h>

#include <rtthread.h>
#include <board.h>
#include <msh.h>

_sig_func_ptr signal(int sig, _sig_func_ptr def)
{
    return NULL;
}

void elua_int_disable_all()
{
}

void elua_int_cleanup()
{
}

void elua_int_enable( elua_int_id inttype )
{
}

void elua_int_disable( elua_int_id inttype )
{
}

int elua_int_add( elua_int_id inttype, elua_int_resnum resnum )
{
  return PLATFORM_ERR;
}

elua_int_c_handler elua_int_set_c_handler( elua_int_id inttype, elua_int_c_handler phandler )
{
  return NULL;
}

elua_int_c_handler elua_int_get_c_handler( elua_int_id inttype )
{
  return NULL;
}

int platform_cpu_set_global_interrupts( int status )
{
  return 0;
}

int platform_cpu_get_global_interrupts( void )
{
  return 0;
}

int linenoise_getline( int id, char* buffer, int maxinput, const char* prompt )
{
    rt_kprintf( prompt );
	
	rt_device_t shell = rt_console_get_device();
	if (shell == NULL) return -1;
	if (buffer == NULL || maxinput <= 1) return -1;
	
	int bufpos = 0;
	while (true){
		// 循环读取
		char ch = 0;
		if (rt_device_read(shell, 0, &ch, 1) <= 0){
			rt_thread_delay(1);
			continue;
		}
		// 退格操作，清理无效字符
		if (ch == 0x7f || ch == 0x08){
			if (bufpos>0){
				bufpos--;
            	rt_kprintf("\b \b");
				buffer[bufpos] = '0';
			}
			continue;
		// 转定义操作，需要循环读取
		}else if (ch == 0x1b){
			char next[10] = {0};
			rt_thread_delay(10);
			rt_device_read(shell, 0, &next, 10);
			continue;
		// 回车操作，需要循环读取
		}else if (ch == '\r'){
			char next;
			rt_thread_delay(10);
			if (rt_device_read(shell, 0, &next, 1) > 0){
				if (next != '\0') ch = next;
			}
		}
		// 判断是否读取完成
		if (bufpos+1 >= maxinput || ch=='\r' || ch=='\n'){
			buffer[bufpos] = '\0';
			// 退出shell
			if (bufpos>=4 && (strncmp(buffer,"exit",4)==0 || strncmp(buffer,"quit",4)==0)){
				return -1;
			}
			// 输出换行
			rt_kprintf("\n");
			return 0;
		}
		// 替换非法字符
		if (ch < ' ' || ch > '~') ch = '`';
		rt_kprintf("%c", ch);
		buffer[bufpos++] = ch;
	}
    return -1;
}

int linenoise_addhistory( int id, const char *line )
{
  return 0;
}

void linenoise_cleanup( int id ) 
{
}

const char* dm_getaddr( int fd )
{
    return NULL;
}
