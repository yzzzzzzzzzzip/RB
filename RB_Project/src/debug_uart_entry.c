#include "debug_uart.h"
#include "bsp_debug_uart.h"
#include "mpu6050.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_api.h" 
#include "math.h"

// 栈溢出钩子函数（保留你的优化）
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    (void)pcTaskName;
    printf("❌ Task[%s] Stack Overflow! System Halted!\r\n", pcTaskName);
    while(1)
    {
        R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS);
    }
}

// 全局变量：校准超时保护用（新增）
TickType_t calib_start;

/* debug_uart entry function：核心改造10ms解算+初始化重试 */
void debug_uart_entry(void * pvParameters)
{
    FSP_PARAMETER_NOT_USED(pvParameters);
    static uint8_t init_flag = 0;
    uint8_t init_retry = 0; // 初始化重试计数器
    Debug_UART_Init();
    Init_I2C_Driver();
		MPU6050_Init_Config();
    // 10ms精准定时用的基准（替换原1000ms）
    TickType_t preTime = xTaskGetTickCount();
    static uint16_t print_cnt = 0;
    UBaseType_t stack_free;

    while(1)
    {
        // ------------------- 初始化循环（失败自动重试） -------------------
        if (init_flag == 0)
        {
            init_retry++;
            printf("\r\n【Init Attempt %d】Start initializing...\r\n", init_retry);
           
            // 3. 零漂校准+初始姿态校准（带超时容错）
            MPU6050_Calibrate();
            MPU6050_Init_Attitude();

            // 初始化成功，标记并打印栈信息
            init_flag = 1;
            stack_free = uxTaskGetStackHighWaterMark(NULL);
            printf("✅ All initialization completed! Enter 10ms calculation loop!\r\n");
            printf("📊 Stack free after init: %lu words = %lu bytes\r\n", 
                   stack_free, stack_free * 4);
            if (stack_free < 100) 
            {
                printf("⚠️ Warning: Insufficient stack space!\r\n");
            }
            // 重置定时基准，保证10ms解算
            preTime = xTaskGetTickCount();
            continue;
        }

        // ------------------- 10ms精准解算循环（初始化成功后） -------------------
        // 核心：vTaskDelayUntil实现10ms固定周期，不漂移
        vTaskDelayUntil(&preTime, pdMS_TO_TICKS(10));

        // 读取六轴数据+姿态解算（10ms一次，和参考代码一致）
        if (MPU6050_Read_SixAxis() == FSP_SUCCESS)
        {
            MPU6050_AHRS_Complementary();

            // 打印分频：每100次解算（100*10ms=1000ms）打印一次
            if (++print_cnt >= 10)
            {
                print_cnt = 0;
                printf("Attitude: Roll=%.1f° Pitch=%.1f°\r\n", Roll, Pitch);
            }
        }
        else
        {
            printf("⚠️ Read six-axis data failed!\r\n");
        }
    }
}