/* generated HAL header file - do not edit */
#ifndef HAL_DATA_H_
#define HAL_DATA_H_
#include <stdint.h>
#include "bsp_api.h"
#include "common_data.h"
#include "r_sci_i2c.h"
#include "r_i2c_master_api.h"
#include "r_sci_uart.h"
            #include "r_uart_api.h"
FSP_HEADER
extern const i2c_master_cfg_t MPU_6050_cfg;
/* I2C on SCI Instance. */
extern const i2c_master_instance_t MPU_6050;
#ifndef MPU_6050_callback
void MPU_6050_callback(i2c_master_callback_args_t * p_args);
#endif

extern const sci_i2c_extended_cfg_t MPU_6050_cfg_extend;
extern sci_i2c_instance_ctrl_t MPU_6050_ctrl;
/** UART on SCI Instance. */
            extern const uart_instance_t      Debug_UART;

            /** Access the UART instance using these structures when calling API functions directly (::p_api is not used). */
            extern sci_uart_instance_ctrl_t     Debug_UART_ctrl;
            extern const uart_cfg_t Debug_UART_cfg;
            extern const sci_uart_extended_cfg_t Debug_UART_cfg_extend;

            #ifndef debug_uart_callback
            void debug_uart_callback(uart_callback_args_t * p_args);
            #endif
void hal_entry(void);
void g_hal_init(void);
FSP_FOOTER
#endif /* HAL_DATA_H_ */
