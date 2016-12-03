#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <rtthread.h>

//#define SHOW_QUE_DEBUG_INFO
//#define SHOW_TIM_DEBUG_INFO
volatile portSTACK_TYPE *pxCurrentTCB[ portNUM_PROCESSORS ] = { NULL };
extern rt_thread_t rt_current_thread;
static unsigned short mq_index = 0;
static unsigned short ms_index = 0;
static unsigned short tm_index = 0;

void rt_hw_context_switch(rt_uint32_t from, rt_uint32_t to)
{
	pxCurrentTCB[0] = (portSTACK_TYPE *)to;
	portYIELD();
}
void rt_hw_context_switch_interrupt(rt_uint32_t from, rt_uint32_t to)
{
	pxCurrentTCB[0] = (portSTACK_TYPE *)to;
	portYIELD_FROM_ISR();
}
rt_base_t rt_hw_interrupt_disable(void)
{
    return portENTER_CRITICAL_NESTED();
}
void rt_hw_interrupt_enable(rt_base_t level)
{
    portEXIT_CRITICAL_NESTED(level);
}

rt_uint8_t *rt_hw_stack_init(void *tentry, void *parameter, rt_uint8_t *stack_addr, void *texit)
{
#if portUSING_MPU_WRAPPERS
	rt_uint8_t *sp = (rt_uint8_t *)pxPortInitialiseStack(stack_addr, tentry, parameter, pdFALSE);
#else
	rt_uint8_t *sp = (rt_uint8_t *)pxPortInitialiseStack(stack_addr, tentry, parameter);
#endif
	return sp;
}

void rtthread_startup(void)
{
    /* disbable interrupt */
    rt_hw_interrupt_disable();

    /* init tick */
    rt_system_tick_init();

    /* init kernel object */
    rt_system_object_init();

    /* init timer system */
    rt_system_timer_init();

    /* init scheduler system */
    rt_system_scheduler_init();

    /* init timer thread */
    rt_system_timer_thread_init();

    /* init idle thread */
    rt_thread_idle_init();
}
void rt_hw_console_output(const char *str)
{
    ets_printf(str);
}

signed portBASE_TYPE xTaskGenericCreate( pdTASK_CODE pxTaskCode, const signed char * const pcName, unsigned short usStackDepth, void *pvParameters, unsigned portBASE_TYPE uxPriority, xTaskHandle *pxCreatedTask, portSTACK_TYPE *puxStackBuffer, const xMemoryRegion * const xRegions )
{
    signed portBASE_TYPE xReturn = errCOULD_NOT_ALLOCATE_REQUIRED_MEMORY;
    rt_thread_t thread = *pxCreatedTask = rt_thread_create((const char *)pcName,pxTaskCode,pvParameters,usStackDepth*sizeof(portSTACK_TYPE),20-uxPriority,10);
    if (thread != 0)
    {
#ifdef SHOW_TSK_DEBUG_INFO
        ets_printf("TaskCreate name:%s pri:%d size:%d\n",pcName,(20-uxPriority),usStackDepth*sizeof(portSTACK_TYPE));
#endif
        rt_thread_startup(thread);
        xReturn = pdPASS;
    }
    return xReturn;
}
void vTaskDelete(xTaskHandle xTaskToDelete)
{
    rt_thread_t thread = xTaskToDelete;
    if (xTaskToDelete == 0)
        thread = rt_current_thread;
#ifdef SHOW_TSK_DEBUG_INFO
    ets_printf("TaskDelete name:%s\n",thread->name);
#endif
    rt_thread_delete(thread);
    rt_schedule();
}
unsigned portBASE_TYPE uxTaskGetStackHighWaterMark( xTaskHandle xTask )
{
    register unsigned short usCount = 0U;
    rt_thread_t thread = xTask;
    if (xTask == 0)
        thread = rt_current_thread;
    rt_uint8_t *ptr = (rt_uint8_t *)thread->stack_addr;
    while (*ptr == '#')
    {
        usCount ++;
        ptr ++;
    }
    usCount /= sizeof( portSTACK_TYPE );
    return usCount;
}
void vTaskStepTick(portTickType xTicksToJump)
{
    ets_printf("StepTick tick:%d cur:%s %d\n",xTicksToJump,rt_current_thread->name,0);
    while (--xTicksToJump > 0)
        rt_tick_increase();
    ets_printf("StepTickOver cur:%s %d\n",rt_current_thread->name,0);
}

void vTaskSwitchContext( void ) { rt_schedule(); }
TickType_t xTaskGetTickCount( void ) { return rt_tick_get(); }
void vTaskSuspendAll( void ) { rt_enter_critical(); }
BaseType_t xTaskResumeAll( void ) { rt_exit_critical();return pdTRUE; }
void vTaskStartScheduler(void) { xPortStartScheduler(); }



void vTaskEnterCritical( portMUX_TYPE *mux )
{
    
}
void vTaskExitCritical( portMUX_TYPE *mux )
{
    
}
BaseType_t xTaskCreatePinnedToCore(	TaskFunction_t pxTaskCode,
							const char * const pcName,
							const uint16_t usStackDepth,
							void * const pvParameters,
							UBaseType_t uxPriority,
							TaskHandle_t * const pxCreatedTask,
                            const BaseType_t xCoreID )
	{
        return 0;
    }
BaseType_t xTaskGetAffinity( TaskHandle_t xTask )
{
	return 0;
}
	char *pcTaskGetTaskName( TaskHandle_t xTaskToQuery ) /*lint !e971 Unqualified char types are allowed for strings and single characters only. */
	{
		return "";
	}
void *rt_malloc(rt_size_t nbytes) { return malloc(nbytes); }
void rt_free(void *ptr) { free(ptr); }
TaskHandle_t xTaskGetCurrentTaskHandleForCPU( BaseType_t cpuid )
{
	return 0;
}
TaskHandle_t xTaskGetCurrentTaskHandle( void )
{
	return 0;
}
BaseType_t xTaskGetSchedulerState( void )
{
	return 0;
}
BaseType_t xTaskIncrementTick( void )
{
	return 0;
}
void vTaskDelay( const TickType_t xTicksToDelay )
{
}
BaseType_t xQueueReceiveFromISR( QueueHandle_t xQueue, void * const pvBuffer, BaseType_t * const pxHigherPriorityTaskWoken )
{
	return 0;
}
BaseType_t xQueueTakeMutexRecursive( QueueHandle_t xMutex, TickType_t xTicksToWait )
{
	return 0;
}
BaseType_t xQueueGiveFromISR( QueueHandle_t xQueue, BaseType_t * const pxHigherPriorityTaskWoken )
{
	return 0;
}
BaseType_t xQueueGiveMutexRecursive( QueueHandle_t xMutex )
{
	return 0;
}
void* xQueueGetMutexHolder( QueueHandle_t xSemaphore )
{
	return 0;
}








extern rt_mailbox_t rt_fmq_create(const char *name, rt_size_t item, rt_size_t size, rt_uint8_t flag);
extern rt_err_t rt_fmq_delete(rt_mailbox_t mb);
extern rt_err_t rt_fmq_send(rt_mailbox_t mb, void *value, rt_int32_t pos, rt_int32_t timeout);
extern rt_err_t rt_fmq_recv(rt_mailbox_t mb, void *value, rt_int32_t peek, rt_int32_t timeout);
xQueueHandle xQueueGenericCreate( unsigned portBASE_TYPE uxQueueLength, unsigned portBASE_TYPE uxItemSize, unsigned char ucQueueType )
{
    char name[10] = {0};
    rt_object_t obj = 0;
    if (uxItemSize <= 0 || uxQueueLength <= 0)
    {
        sprintf(name,"s%02d",((++ms_index)%100));
        obj = (rt_object_t)rt_sem_create(name,0,RT_IPC_FLAG_PRIO);
    }
    else
    {
        sprintf(name,"q%02d",((++mq_index)%100));
        obj = (rt_object_t)rt_fmq_create(name,uxItemSize,uxQueueLength,RT_IPC_FLAG_PRIO);
    }
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueCreate name:%s count:%d size:%d\n",name,uxQueueLength,uxItemSize);
#endif
    return obj;
}
xQueueHandle xQueueCreateMutex( unsigned char ucQueueType )
{
    xQueueHandle obj = xQueueGenericCreate(1,0,ucQueueType);
    if (obj)
        xQueueGenericSend(obj,0,0,queueSEND_TO_BACK);
    return obj;
}
void vQueueDelete( xQueueHandle xQueue )
{
    rt_object_t obj = xQueue;
    if (obj == 0)
        return;
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueDelete name:%s\n",obj->name);
#endif
    if (obj->type == RT_Object_Class_Semaphore)
        rt_sem_delete((rt_sem_t)obj);
    else
        rt_fmq_delete((rt_mailbox_t)obj);
}
signed portBASE_TYPE xQueueGenericSend( xQueueHandle xQueue, const void * const pvItemToQueue, portTickType xTicksToWait, portBASE_TYPE xCopyPosition )
{
    rt_object_t obj = xQueue;
    if (obj == 0)
        return errQUEUE_FULL;
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueSend cur:%s name:%s wait:%d pos:%d\n",rt_current_thread->name,obj->name,xTicksToWait,xCopyPosition);
#endif
    rt_err_t err = RT_EOK;
    if (obj->type == RT_Object_Class_Semaphore)
        err = rt_sem_release((rt_sem_t)obj);
    else
        err = rt_fmq_send((rt_mailbox_t)obj,(void *)pvItemToQueue,xCopyPosition,xTicksToWait);
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueSendOver cur:%s name:%s ret:%d\n",rt_current_thread->name,obj->name,err);
#endif
    return (err==RT_EOK)?pdPASS:errQUEUE_FULL;
}
signed portBASE_TYPE xQueueGenericSendFromISR( xQueueHandle xQueue, const void * const pvItemToQueue, signed portBASE_TYPE *pxHigherPriorityTaskWoken, portBASE_TYPE xCopyPosition )
{
    rt_object_t obj = xQueue;
    if (obj == 0)
        return errQUEUE_FULL;
    rt_interrupt_enter();
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueSendISR cur:%s name:%s pos:%d\n",rt_current_thread->name,obj->name,xCopyPosition);
#endif
    rt_err_t err = RT_EOK;
    if (obj->type == RT_Object_Class_Semaphore)
        err = rt_sem_release((rt_sem_t)obj);
    else
        err = rt_fmq_send((rt_mailbox_t)obj,(void *)pvItemToQueue,xCopyPosition,0);
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueSendISROver cur:%s name:%s ret:%d\n",rt_current_thread->name,obj->name,err);
#endif
    if (pxHigherPriorityTaskWoken) *pxHigherPriorityTaskWoken = pdFAIL;
    rt_interrupt_leave();
    return (err==RT_EOK)?pdPASS:errQUEUE_FULL;
}
BaseType_t xQueueGenericReceive( QueueHandle_t xQueue, void * const pvBuffer, TickType_t xTicksToWait, const BaseType_t xJustPeek )
{
    rt_object_t obj = xQueue;
    if (obj == 0)
        return errQUEUE_EMPTY;
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueRecv cur:%s name:%s wait:%d peek:%d\n",rt_current_thread->name,obj->name,xTicksToWait,xJustPeek);
#endif
    rt_err_t err = RT_EOK;
    if (obj->type == RT_Object_Class_Semaphore)
        err = rt_sem_take((rt_sem_t)obj,xTicksToWait);
    else
        err = rt_fmq_recv((rt_mailbox_t)obj,(void *)pvBuffer,xJustPeek,xTicksToWait);
#ifdef SHOW_QUE_DEBUG_INFO
    ets_printf("QueueRecvOver cur:%s name:%s ret:%d\n",rt_current_thread->name,obj->name,err);
#endif
    return (err==RT_EOK)?pdPASS:errQUEUE_EMPTY;
}
unsigned portBASE_TYPE uxQueueMessagesWaiting(const xQueueHandle xQueue)
{ 
    rt_object_t obj = xQueue;
    if (obj == 0)
        return 0;
    unsigned portBASE_TYPE count = 0;
    //vPortEnterCritical();
    if (obj->type == RT_Object_Class_Semaphore)
        count = ((rt_sem_t)obj)->value;
    else
        count = ((rt_mailbox_t)obj)->entry;
    //vPortExitCritical();
    return count;
}
unsigned portBASE_TYPE uxQueueMessagesWaitingFromISR(const xQueueHandle xQueue)
{ 
    rt_object_t obj = xQueue;
    if (obj == 0)
        return 0;
    unsigned portBASE_TYPE count = 0;
    if (obj->type == RT_Object_Class_Semaphore)
        count = ((rt_sem_t)obj)->value;
    else
        count = ((rt_mailbox_t)obj)->entry;
    return count;
}

TimerHandle_t xTimerCreate( const char * const pcTimerName, const TickType_t xTimerPeriodInTicks, const UBaseType_t uxAutoReload, void * const pvTimerID, TimerCallbackFunction_t pxCallbackFunction )
{
    char name[10] = {0},*tname = (char *)pcTimerName;
    rt_timer_t obj = 0;
    if (tname == 0)
    {
        sprintf(name,"t%02d",((++tm_index)%100));
        tname = name;
    }
    if (xTimerPeriodInTicks > 0)
    {
        rt_uint8_t flag = (uxAutoReload == pdTRUE)?(RT_TIMER_FLAG_PERIODIC):(RT_TIMER_FLAG_ONE_SHOT);
        obj = rt_timer_create(tname,pxCallbackFunction,pvTimerID,xTimerPeriodInTicks,flag);
    }
#ifdef SHOW_TIM_DEBUG_INFO
    ets_printf("xTimerCreate name:%s tick:%d auto:%d id:%x func:%x\n",tname,xTimerPeriodInTicks,uxAutoReload,pvTimerID,pxCallbackFunction);
#endif
    return obj;
}
portBASE_TYPE xTimerGenericCommand( xTimerHandle xTimer, portBASE_TYPE xCommandID, portTickType xOptionalValue, signed portBASE_TYPE *pxHigherPriorityTaskWoken, portTickType xBlockTime )
{
    rt_timer_t obj = xTimer;
    if (obj == 0)
        return pdFAIL;
    if (pxHigherPriorityTaskWoken)
        rt_interrupt_enter();
#ifdef SHOW_TIM_DEBUG_INFO
    ets_printf("xTimerCommand cur:%s name:%s cmd:%d val:%d tim:%d\n",rt_current_thread->name,obj->parent.name,xCommandID,xOptionalValue,xBlockTime);
#endif
    rt_err_t err = RT_EOK;
    switch(xCommandID)
    {
    case tmrCOMMAND_START:
        err = rt_timer_start(obj);
        break;
    case tmrCOMMAND_STOP:
        err = rt_timer_stop(obj);
        break;
    case tmrCOMMAND_CHANGE_PERIOD:{
        rt_tick_t tick = xOptionalValue;
        err = rt_timer_control(obj,RT_TIMER_CTRL_SET_TIME,&tick);
        break;}
    case tmrCOMMAND_DELETE:
        err = rt_timer_delete(obj);
        break;
    }
#ifdef SHOW_TIM_DEBUG_INFO
    ets_printf("xTimerCommandOver cur:%s name:%s ret:%d\n",rt_current_thread->name,obj->parent.name,err);
#endif
    if (pxHigherPriorityTaskWoken) {
        *pxHigherPriorityTaskWoken = pdFAIL;
        rt_interrupt_leave();
    }
    return pdTRUE;
}
