#include "debug_uart.h"
#include "bsp_debug_uart.h"
                /* debug_uart entry function */
                /* pvParameters contains TaskHandle_t */
                void debug_uart_entry(void * pvParameters)
                {
                    FSP_PARAMETER_NOT_USED(pvParameters);
										
                    /* TODO: add your own code here */
										Debug_UART_Init();
                    while(1)
                    {
											printf("hello world");
                        vTaskDelay(5000);
                    }
                }
