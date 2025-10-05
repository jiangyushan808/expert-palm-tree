#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#include "stm32f4xx.h"

/* 芯片特定配置 */
#define configCPU_CLOCK_HZ                  ( 84000000 )     // F401主频84MHz
#define configTICK_RATE_HZ                  ( 1000 )         // 系统节拍1kHz
#define configSYSTICK_CLOCK_HZ              ( configCPU_CLOCK_HZ )

/* 内存配置 - F401只有64KB SRAM */
#define configTOTAL_HEAP_SIZE               ( ( size_t ) ( 20 * 1024 ) )
#define configMINIMAL_STACK_SIZE            ( ( unsigned short ) 128 )
#define configMAX_TASK_NAME_LEN             ( 16 )

/* 任务配置 */
#define configMAX_PRIORITIES                ( 7 )
#define configUSE_PREEMPTION                1
#define configUSE_TIME_SLICING              1
#define configIDLE_SHOULD_YIELD             1

/* 同步原语 */
#define configUSE_MUTEXES                   1
#define configUSE_RECURSIVE_MUTEXES         1
#define configUSE_COUNTING_SEMAPHORES       1
#define configUSE_QUEUE_SETS                0    // 节省内存

/* 内存管理 */
#define configSUPPORT_DYNAMIC_ALLOCATION    1
#define configSUPPORT_STATIC_ALLOCATION     0
#define configAPPLICATION_ALLOCATED_HEAP    0

/* Hook函数 */
#define configUSE_IDLE_HOOK                 0
#define configUSE_TICK_HOOK                 0
#define configCHECK_FOR_STACK_OVERFLOW      2    // 栈溢出检查

/* 调试统计 */
#define configGENERATE_RUN_TIME_STATS       0    // 节省资源
#define configUSE_TRACE_FACILITY            0
#define configUSE_STATS_FORMATTING_FUNCTIONS 0

/* 协程（已废弃） */
#define configUSE_CO_ROUTINES               0
#define configMAX_CO_ROUTINE_PRIORITIES     ( 2 )

/* 软件定时器 */
#define configUSE_TIMERS                    1
#define configTIMER_TASK_PRIORITY           ( 3 )
#define configTIMER_QUEUE_LENGTH            ( 10 )
#define configTIMER_TASK_STACK_DEPTH        ( configMINIMAL_STACK_SIZE * 2 )

/* 中断配置 */
#define configPRIO_BITS                         ( 4 )  // Cortex-M4 有 4 位优先级
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY ( 15 )
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY ( 5 )

#define configKERNEL_INTERRUPT_PRIORITY         ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )
#define configMAX_SYSCALL_INTERRUPT_PRIORITY    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

/* 中断服务程序映射 */

#define xPortSysTickHandler                 SysTick_Handler

/* 断言配置 */

#define configASSERT(x) if( (x) == 0 ) vAssertCalled(__FILE__, __LINE__)
	

#define xPortPendSVHandler  PendSV_Handler
#define vPortSVCHandler     SVC_Handler
#define INCLUDE_xTaskGetSchedulerState   1

#define configUSE_16_BIT_TICKS    0   // 对于 STM32F4 必须是 0（32bit tick）



#endif /* FREERTOS_CONFIG_H */
