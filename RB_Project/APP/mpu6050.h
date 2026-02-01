#ifndef __MPU6050_H
#define __MPU6050_H

#include "hal_data.h"
#include "stdint.h"
#include "stdio.h"
#include "string.h"
#include "FreeRTOS.h"
#include "task.h"
#ifndef M_PI
#define M_PI 3.1415926535f  // 浮点型f，适配float计算，避免精度损失
#endif
#include <math.h>

// ************************* 函数声明 *************************
fsp_err_t Init_I2C_Driver(void);  // MPU6050初始化（解除睡眠、配置默认参数）
fsp_err_t MPU6050_WriteReg(uint8_t reg, uint8_t data);
fsp_err_t MPU6050_ReadRegs(unsigned char* ptr_read, uint8_t reg, uint8_t len);
fsp_err_t MPU6050_Init_Config(void);
fsp_err_t MPU6050_Read_SixAxis(void);
void MPU6050_Calibrate(void);
void MPU6050_Init_Attitude(void);

// 全局变量声明（六轴物理值+姿态角）
extern float AccelX_g, AccelY_g, AccelZ_g;
extern float GyroX_dps, GyroY_dps, GyroZ_dps;
extern float MPU6050_Temp;
extern float Roll, Pitch, Yaw;


	/*	// 主循环：10ms一次读取+解算（100Hz），100ms一次打印（10Hz）
		if (HAL_GetTick() - last_time >= 10) // 10ms一次（100Hz解算，不修改）
		{
				// 读取六轴+温度数据（带零漂补偿）
				MPU6050_Read_SixAxis();
				// 互补滤波姿态解算（动态系数+静态校准）
				MPU6050_AHRS_Complementary();
		
				// ------------------- 新增打印分频逻辑（仅3行） -------------------
				static uint16_t print_cnt = 0; // 静态计数器，仅初始化一次，每次解算自增
				if (++print_cnt >= 200) // 累计10次解算（10*10ms=100ms）打印一次，可改阈值
				{
						print_cnt = 0; // 重置计数器
						// 串口打印数据（仅满足条件时执行，降低打印频率）
						printf("Acc: X=%.2fg Y=%.2fg Z=%.2fg | 姿态角：Roll=%.1f° Pitch=%.1f°\r\n",
									AccelX_g, AccelY_g, AccelZ_g, Roll, Pitch);
				}
				// -------------------------------------------------------------------
		
				// 更新时间基准（必须保留，保证100Hz解算）
				last_time = HAL_GetTick();
		}  */
		
				// R_BSP_SoftwareDelay(1, BSP_DELAY_UNITS_SECONDS); //延时1秒


#endif // __MPU6050_H
