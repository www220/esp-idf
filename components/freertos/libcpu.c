#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>
#include <errno.h>

#include "rom/ets_sys.h"
#include "esp_newlib.h"
#include "event_groups.h"
#include "thread_esp32.h"
#include "esp_heap_caps.h"
#include "semphr.h"

extern void rt_hw_board_init();
void rtthread_startup(void)
{
    /* init board */
    rt_hw_board_init();

    /* show version */
	rt_kprintf("\n\n");
    rt_show_version();

    /* init kernel object */
    rt_system_object_init();
}

rt_err_t rt_sem_take(rt_sem_t sem, rt_int32_t time)
{
    if (xPortInIsrContext()){
        configASSERT(time!=0);
        BaseType_t higher_task_woken = pdFALSE;
        BaseType_t ret = xSemaphoreTakeFromISR(sem, &higher_task_woken);
        if (higher_task_woken==pdTRUE) portYIELD_FROM_ISR();
        return ret==pdPASS?RT_EOK:-RT_ERROR;
    }
    return xSemaphoreTake(sem,time)==pdPASS?RT_EOK:-RT_ETIMEOUT;
}

rt_err_t rt_sem_release(rt_sem_t sem)
{
    if (xPortInIsrContext()){
        BaseType_t higher_task_woken = pdFALSE;
        BaseType_t ret = xSemaphoreGiveFromISR(sem, &higher_task_woken);
        if (higher_task_woken==pdTRUE) portYIELD_FROM_ISR();
        return ret==pdPASS?RT_EOK:-RT_ERROR;
    }
    return xSemaphoreGive(sem)==pdPASS?RT_EOK:-RT_ERROR;
}

rt_err_t rt_thread_init(struct rt_thread *thread, const char *name, void (*entry)(void *parameter), void *parameter, void *stack_start, rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick)
{
    vTaskSuspendAll();
    rt_thread_t tid = xTaskCreateStaticPinnedToCore(entry, name, stack_size, parameter, RT_THREAD_PRIORITY_MAX-1-priority, stack_start, thread, 0);
    if (tid!=NULL) vTaskSuspend(tid);
    xTaskResumeAll();
    return tid!=NULL?RT_EOK:-RT_ERROR;
}

rt_thread_t rt_thread_create(const char *name, void (*entry)(void *parameter), void *parameter, rt_uint32_t stack_size, rt_uint8_t priority, rt_uint32_t tick)
{
    vTaskSuspendAll();
    rt_thread_t tid = NULL;
    BaseType_t ret = xTaskCreatePinnedToCore(entry, name, stack_size, parameter, RT_THREAD_PRIORITY_MAX-1-priority, &tid, 0);
    if (ret!=pdPASS) tid = NULL;
    if (tid!=NULL) vTaskSuspend(tid);
    xTaskResumeAll();
    return tid;
}

void rt_memory_info(rt_uint32_t *total, rt_uint32_t *used, rt_uint32_t *max_used)
{
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);

    if (total != RT_NULL)
        *total = info.total_allocated_bytes+info.total_free_bytes;
    if (used  != RT_NULL)
        *used = info.total_allocated_bytes;
    if (max_used != RT_NULL)
        *max_used = info.total_allocated_bytes+info.total_free_bytes-info.minimum_free_bytes;
}

void list_mem(void)
{
    multi_heap_info_t info;
    heap_caps_get_info(&info, MALLOC_CAP_8BIT);

    rt_kprintf("total memory: %d\n", info.total_allocated_bytes+info.total_free_bytes);
    rt_kprintf("used memory : %d\n", info.total_allocated_bytes);
    rt_kprintf("maximum allocated memory: %d\n", info.total_allocated_bytes+info.total_free_bytes-info.minimum_free_bytes);
}

rt_err_t rt_mutex_init(rt_mutex_t mutex, const char *name, rt_uint8_t flag) { xSemaphoreCreateMutexStatic(mutex);return RT_EOK; }
rt_err_t rt_mutex_take(rt_mutex_t mutex, rt_int32_t time) { return xSemaphoreTake(mutex,time)==pdPASS?RT_EOK:-RT_ETIMEOUT; }
rt_err_t rt_mutex_release(rt_mutex_t mutex) { return xSemaphoreGive(mutex)==pdPASS?RT_EOK:-RT_ERROR; }
rt_err_t rt_sem_init(rt_sem_t sem, const char *name, rt_uint32_t value, rt_uint8_t  flag) { xSemaphoreCreateCountingStatic(0xffffffff,value,sem);return RT_EOK; }
rt_err_t rt_thread_startup(rt_thread_t thread) { vTaskResume(thread);return RT_EOK; }
rt_err_t rt_thread_suspend(rt_thread_t thread) { vTaskSuspend(thread);return RT_EOK; }
rt_err_t rt_thread_resume(rt_thread_t thread) { vTaskResume(thread);return RT_EOK; }
rt_err_t rt_thread_delete(rt_thread_t thread) { vTaskDelete(thread);return RT_EOK; }
rt_thread_t rt_thread_self(void) { return xTaskGetCurrentTaskHandle(); }
rt_err_t rt_thread_delay(rt_tick_t tick) { vTaskDelay(tick);return RT_EOK; }
void rt_enter_critical(void) { vTaskSuspendAll(); }
void rt_exit_critical(void) { xTaskResumeAll(); }
rt_base_t rt_hw_interrupt_disable(void) { return portENTER_CRITICAL_NESTED(); }
void rt_hw_interrupt_enable(rt_base_t level) { portEXIT_CRITICAL_NESTED(level); }
rt_err_t rt_get_errno(void) { return *__errno(); }
void rt_set_errno(rt_err_t error) { *__errno() = error; }
int *_rt_errno(void) { return __errno(); }
rt_tick_t rt_tick_get(void) { return xTaskGetTickCount(); }
void *rt_malloc(rt_size_t nbytes) { return pvPortMalloc(nbytes); }
void rt_free(void *ptr) { vPortFree(ptr); }
