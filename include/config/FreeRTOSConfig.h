/*
 * FreeRTOS Kernel V10.2.0
 * Copyright (C) 2017 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://aws.amazon.com/freertos
 * http://www.FreeRTOS.org
*/
/*
 * Copyright (C) 2011-2019, Marvell International Ltd.
 * All Rights Reserved.
 */

#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

/*-----------------------------------------------------------
 * Application specific definitions.
 *
 * These definitions should be adjusted for your particular hardware and
 * application requirements.
 *
 * THESE PARAMETERS ARE DESCRIBED WITHIN THE 'CONFIGURATION' SECTION OF THE
 * FreeRTOS API DOCUMENTATION AVAILABLE ON THE FreeRTOS.org WEB SITE. 
 *
 * See http://www.freertos.org/a00110.html.
 *----------------------------------------------------------*/

#define configUSE_PREEMPTION			1
#define configUSE_IDLE_HOOK			1
#define configUSE_TICK_HOOK			0
/* #define configCPU_CLOCK_HZ			( ( unsigned long ) 200000000 ) */
#define configCPU_CLOCK_HZ              	( board_cpu_freq( ) )
#define configTICK_RATE_HZ			( (TickType_t) 1000 )
#define configMAX_PRIORITIES			( 7 )
#define configMINIMAL_STACK_SIZE		( ( unsigned short ) 256 )
/* #define configTOTAL_HEAP_SIZE		( ( size_t ) ( 72 * 1024 ) ) */
#define configMAX_TASK_NAME_LEN			( 16 )
#define configUSE_TRACE_FACILITY		1
#define configUSE_STATS_FORMATTING_FUNCTIONS	1
#define configUSE_16_BIT_TICKS			0
#define configIDLE_SHOULD_YIELD			1
#define configUSE_MUTEXES			1
#if ( CONFIG_ENABLE_STACK_OVERFLOW_CHECK == 1 )
#define configCHECK_FOR_STACK_OVERFLOW		2
#else
#define configCHECK_FOR_STACK_OVERFLOW  	0
#endif
#define configUSE_MALLOC_FAILED_HOOK		1
#define configUSE_RECURSIVE_MUTEXES		1
#define configUSE_COUNTING_SEMAPHORES		1

/* Currently the TCP/IP stack is using dynamic allocation, and the MQTT task is
 * using static allocation. */
#define configSUPPORT_DYNAMIC_ALLOCATION        1
#define configSUPPORT_STATIC_ALLOCATION         1

/* Assert call defined for debug builds. */
void vAssertCalled( const char * pcFile,
                    uint32_t ulLine );

/* Board CPU frequency */
int board_cpu_freq();

#define configASSERT( x )    if( ( x ) == 0 ) vAssertCalled( __FILE__, __LINE__ )

/* Enable this if run time statistics are to be enabled. The support
   functions are already added in WMSDK */
#if ( CONFIG_ENABLE_RUNTIME_STATS == 1 )
#define configGENERATE_RUN_TIME_STATS   	1
#else
#define configGENERATE_RUN_TIME_STATS   	0
#endif

#if ( configGENERATE_RUN_TIME_STATS == 1 )
/* wmsdk: Prototype of function defined in wmsdk */
void portWMSDK_CONFIGURE_TIMER_FOR_RUN_TIME_STATS(void);
unsigned long portWMSDK_GET_RUN_TIME_COUNTER_VALUE(void);

#define portCONFIGURE_TIMER_FOR_RUN_TIME_STATS		\
	portWMSDK_CONFIGURE_TIMER_FOR_RUN_TIME_STATS
#define portGET_RUN_TIME_COUNTER_VALUE		\
	portWMSDK_GET_RUN_TIME_COUNTER_VALUE
#endif /* configGENERATE_RUN_TIME_STATS */

#define configUSE_CO_ROUTINES 			0
#define configMAX_CO_ROUTINE_PRIORITIES 	( 2 )

/* Use application defined heap region */
#define configAPPLICATION_ALLOCATED_HEAP	0

/*  Software timer definitions. */
#define configUSE_TIMERS			1
#define configTIMER_TASK_PRIORITY		( configMAX_PRIORITIES - 1  )
#define configTIMER_QUEUE_LENGTH		5
#define configTIMER_TASK_STACK_DEPTH		( configMINIMAL_STACK_SIZE )

/* Set the following definitions to 1 to include the API function, or zero
to exclude the API function. */
#define INCLUDE_vTaskPrioritySet		1
#define INCLUDE_uxTaskPriorityGet		1
#define INCLUDE_vTaskDelete			1
#define INCLUDE_vTaskCleanUpResources		0
#define INCLUDE_vTaskSuspend			1
#define INCLUDE_vTaskDelayUntil			1
#define INCLUDE_vTaskDelay			1
#define INCLUDE_xTaskGetSchedulerState          1
#define INCLUDE_xTaskGetCurrentTaskHandle       1
#define INCLUDE_uxTaskGetStackHighWaterMark     1
#define INCLUDE_xTaskGetIdleTaskHandle          1
#define INCLUDE_eTaskGetState                   1
#define INCLUDE_xEventGroupSetBitFromISR        1
#define INCLUDE_xTimerPendFunctionCall          1
#define INCLUDE_xTaskAbortDelay                 1
#define INCLUDE_xTaskGetHandle                  1
#define INCLUDE_xTaskResumeFromISR              1
#define INCLUDE_xQueueGetMutexHolder            1

#define configKERNEL_INTERRUPT_PRIORITY 	0xf0
#define configMAX_SYSCALL_INTERRUPT_PRIORITY 	0xa0 /* equivalent to 0xa0, or priority 5. */

#define xPortPendSVHandler	PendSV_IRQHandler
#define xPortSysTickHandler	SysTick_IRQHandler
#define vPortSVCHandler		SVC_IRQHandler

#define configPROFILING                      ( 0 )

/* Pseudo random number generater used by some demo tasks. */
extern uint32_t ulRand();
#define configRAND32()    ulRand()

/* FIX ME: The platform FreeRTOS is running on. */
#define configPLATFORM_NAME    "MW320"

/* Header required for the tracealyzer recorder library. */
/* #include "trcRecorder.h" */

#define ENABLE_LOGGING
#ifdef ENABLE_LOGGING
/* The function that implements FreeRTOS printf style output, and the macro
 * that maps the configPRINTF() macros to that function. */
extern void vLoggingPrintf( const char * pcFormat, ... );
#define configPRINTF( X )    vLoggingPrintf X

/* Non-format version thread-safe print */
extern void vLoggingPrint( const char * pcMessage );
#define configPRINT( X )     vLoggingPrint( X )
#else
#define configPRINTF( X )
#define configPRINT( X )
#endif

/* Map the logging task's printf to the board specific output function. */
void ll_printf(const char *format, ...);
#define configPRINT_STRING( X )    ll_printf( X ); /* FIX ME: Change to your devices console print acceptance function. */
/* Sets the length of the buffers into which logging messages are written - so
 * also defines the maximum length of each log message. */
#define configLOGGING_MAX_MESSAGE_LENGTH            128

/* Set to 1 to prepend each log message with a message number, the task name,
 * and a time stamp. */
#define configLOGGING_INCLUDE_TIME_AND_TASK_NAME    1

/* The size of the global output buffer that is available for use when there
   *  are multiple command interpreters running at once (for example, one on a UART
   *  and one on TCP/IP).  This is done to prevent an output buffer being defined by
   *  each implementation - which would waste RAM.  In this case, there is only one
   *  command interpreter running, and it has its own local output buffer, so the
   *  global buffer is just set to be one byte long as it is not used and should not
   *  take up unnecessary RAM. */
#define configCOMMAND_INT_MAX_OUTPUT_SIZE           1

#define max(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a > _b ? _a : _b;       \
})

#define min(a,b)             \
({                           \
    __typeof__ (a) _a = (a); \
    __typeof__ (b) _b = (b); \
    _a < _b ? _a : _b;       \
})

#define configMAX(a,b) max(a,b)
#define configMIN(a,b) min(a,b)

#endif /* FREERTOS_CONFIG_H */
