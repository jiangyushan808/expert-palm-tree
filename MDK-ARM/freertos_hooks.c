#include "FreeRTOS.h"
#include "task.h"

// 栈溢出钩子
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    // 可以打印任务名或停机调试
    (void)xTask;
    (void)pcTaskName;
    while(1);
}

// malloc失败钩子
void vApplicationMallocFailedHook(void)
{
    while(1);
}

// assert 钩子
void vAssertCalled(const char *file, int line)
{
    (void)file;
    (void)line;
    while(1);
}
