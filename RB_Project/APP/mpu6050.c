#include "mpu6050.h"
#include "bsp_debug_uart.h"  // 复用串口打印（已实现printf重定向）
#include "hal_data.h"        // 必须包含，获取SCI-I2C/事件类型定义

/************************* 全局变量（保留你的命名习惯，加volatile防编译器优化） *************************/
uint8_t tx_buf[2];                // 通用发送缓冲区：[0]寄存器地址 + [1]写入数据（I2C写固定格式）
extern sci_i2c_instance_ctrl_t MPU_6050_ctrl;  // 复用SCI-I2C控制结构体（类型匹配）
extern const i2c_master_cfg_t MPU_6050_cfg; // MPU6050 I2C配置结构体（RASC生成）
volatile unsigned int timeout_ms = 500;         // 超时计数器（加volatile，回调/主函数共用）
volatile i2c_master_event_t MPU_6050_callback_event; // 回调事件标志（加volatile，防止优化）

/************************* 全局变量：六轴原始数据+物理值（供姿态解算用） *************************/
// 原始16位数据（MPU6050输出为16位有符号数）
int16_t MPU6050_AccelX, MPU6050_AccelY, MPU6050_AccelZ;
int16_t MPU6050_GyroX,  MPU6050_GyroY,  MPU6050_GyroZ;
// 物理值（加速度：g；陀螺仪：°/s；温度：℃）
float AccelX_g, AccelY_g, AccelZ_g;
float GyroX_dps, GyroY_dps, GyroZ_dps;
float MPU6050_Temp;

/************************* 姿态解算全局变量 *************************/
float Roll = 0.0f, Pitch = 0.0f, Yaw = 0.0f; // 横滚/俯仰/偏航角（单位：°）
uint32_t last_time = 0;                       // 上一次解算时间（SysTick，ms）
float COMPLEMENTARY_COEFF = 0.98f;           // 互补滤波融合系数（0.98最优）

/************************* MPU6050零漂校准（静态时执行） *************************/
float Accel_Offset[3] = {0.0f, 0.0f, 0.0f}; // 加速度计零漂偏移
float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};  // 陀螺仪零漂偏移

/************************* I2C主设备回调函数（适配MPU6050，保留你的逻辑） *************************/
void MPU_6050_callback(i2c_master_callback_args_t * p_args)
{
    // 空指针校验，防止程序崩溃
    if (NULL != p_args)
    {
        MPU_6050_callback_event = p_args->event; // 捕获I2C传输事件
    }
}

/************************* SCI-I2C驱动初始化（保留你的核心逻辑） *************************/
fsp_err_t Init_I2C_Driver(void)
{
    fsp_err_t err = FSP_SUCCESS;

    /* 打开SCI_I2C主设备通道（适配你的控制/配置结构体） */
    err = R_SCI_I2C_Open(&MPU_6050_ctrl, &MPU_6050_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("SCI_I2C主设备打开失败！Err=%d \r\n", err);
        return err;
    }

    printf("SCI_I2C主设备初始化成功！\r\n");
    return err;
}

/************************* 私有函数：MPU6050单寄存器写（修正重定义，适配SCI-I2C） *************************/
// 修正：原函数重定义，改名MPU6050_WriteReg；补data形参；修复逻辑顺序
 fsp_err_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    fsp_err_t err = FSP_SUCCESS;

    // 1. 填充发送缓冲区：寄存器地址 + 要写入的数据
    tx_buf[0] = reg;
    tx_buf[1] = data;

    // 2. 重置回调事件标志（核心！防止上一次事件污染，导致等待循环直接跳过）
    //MPU_6050_callback_event = I2C_MASTER_EVENT_NONE;

    // 3. 启动SCI-I2C异步写操作（统一API，适配SCI-I2C）
    // 参数：控制结构体 | 发送缓冲区 | 长度2字节(地址+数据) | restart=false(写完释放总线)
    err = R_SCI_I2C_Write(&MPU_6050_ctrl, tx_buf, 2, false);
    if (FSP_SUCCESS != err)
    {
        printf("MPU6050写寄存器失败！Reg=0x%02X, Err=%d\r\n", reg, err);
        return err;
    }

    // 4. 异步等待写完成事件 + 超时保护（修正顺序：移到return前，保证执行）
    // 加400μs延时：降低CPU占用，避免空转（小幅度优化关键）
    while ((I2C_MASTER_EVENT_TX_COMPLETE != MPU_6050_callback_event) && timeout_ms)
    {
        R_BSP_SoftwareDelay(400U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
    }

    // 5. 超时判断：打印错误信息，便于调试
    if (timeout_ms == 0)
    {
        printf("MPU6050写寄存器超时！Reg=0x%02X\r\n", reg);
        err = FSP_ERR_TIMEOUT; // 置超时错误码，供上层判断
    }

    // 6. 重置超时计数器 + 清空发送缓冲区（为下一次操作做准备）
    timeout_ms = 500;
    memset(tx_buf, 0, sizeof(tx_buf));

    return err;
}

/************************* 私有函数：MPU6050多寄存器读（修正API/逻辑，补全核心步骤） *************************/
// 修正：统一SCI-I2C API；补事件重置；加延时；简化缓冲区；保留你的核心逻辑
 fsp_err_t MPU6050_ReadRegs(unsigned char* ptr_read, uint8_t reg, uint8_t len)
{
    fsp_err_t err = FSP_SUCCESS;



    // 1. 填充要读取的寄存器地址（复用全局tx_buf，减少冗余变量）
    tx_buf[0] = reg;

    // 2. 重置回调事件标志（核心！避免上一次事件影响）
    //MPU_6050_callback_event = I2C_MASTER_EVENT_NONE;

    // 3. 启动SCI-I2C写地址操作（统一API，适配SCI-I2C）
    // 参数：restart=true(写完地址不释放总线，直接读数据，I2C连续传输关键)
    err = R_SCI_I2C_Write(&MPU_6050_ctrl, tx_buf, 1, true);
    if (FSP_SUCCESS != err)
    {
        printf("MPU6050发寄存器地址失败！Reg=0x%02X, Err=%d\r\n", reg, err);
        return err;
    }

    // 4. 异步等待写地址完成事件 + 超时保护（加延时，降低CPU占用）
    while ((I2C_MASTER_EVENT_TX_COMPLETE != MPU_6050_callback_event) && timeout_ms)
    {
        R_BSP_SoftwareDelay(400U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
    }

    // 5. 写地址超时判断
    if (timeout_ms == 0)
    {
        printf("MPU6050发寄存器地址超时！Reg=0x%02X\r\n", reg);
        err = FSP_ERR_TIMEOUT;
        timeout_ms = 500; // 重置超时，避免影响后续操作
        return err;
    }

    // 6. 重置超时计数器 + 事件标志（为读操作做准备）
    timeout_ms = 500;
    //MPU_6050_callback_event = I2C_MASTER_EVENT_NONE;

    // 7. 启动SCI-I2C异步读操作（统一API，适配SCI-I2C）
    // 参数：restart=false(读完释放总线，允许其他I2C设备使用)
    err = R_SCI_I2C_Read(&MPU_6050_ctrl, ptr_read, len, false);
    if (FSP_SUCCESS != err)
    {
        printf("SCI_I2C主设备读API执行失败！Reg=0x%02X, Err=%d\r\n", reg, err);
        return err;
    }

    // 8. 异步等待读完成事件 + 超时保护（核心补充！原代码缺失，防止数据不全）
    while ((I2C_MASTER_EVENT_RX_COMPLETE != MPU_6050_callback_event) && timeout_ms)
    {
        R_BSP_SoftwareDelay(400U, BSP_DELAY_UNITS_MICROSECONDS);
        timeout_ms--;
    }

    // 9. 读操作超时判断
    if (timeout_ms == 0)
    {
        printf("MPU6050读寄存器超时！Reg=0x%02X\r\n", reg);
        err = FSP_ERR_TIMEOUT;
    }

    // 10. 重置超时计数器，为下一次操作做准备
    timeout_ms = 500;

    return err;
}
/************************* MPU6050高灵敏+高稳定初始化配置 *************************/
fsp_err_t MPU6050_Init_Config(void)
{
    fsp_err_t err = FSP_SUCCESS;
    // 1. 唤醒MPU6050（0x6B=电源管理寄存器，0x00=解除睡眠，开启内部8MHz晶振）
    err = MPU6050_WriteReg(0x6B, 0x00);
    if (err != FSP_SUCCESS) return err;
    R_BSP_SoftwareDelay(100U, BSP_DELAY_UNITS_MILLISECONDS); // 硬件稳定延时

    // 2. 配置加速度计：±2g量程 + 高分辨率（0x1C=加速度计配置寄存器，0x00=±2g）
    err = MPU6050_WriteReg(0x1C, 0x00);
    if (err != FSP_SUCCESS) return err;

    // 3. 配置陀螺仪：±250°/s量程 + 高分辨率（0x1B=陀螺仪配置寄存器，0x00=±250°/s）
    err = MPU6050_WriteReg(0x1B, 0x00);
    if (err != FSP_SUCCESS) return err;

    // 4. 配置采样率+低通滤波：100Hz采样 + 41Hz DLPF（核心！平衡灵敏+稳定）
    // 0x19=采样率分频寄存器，SAMPLE_RATE=8000/(SMPLRT_DIV+1)，79→100Hz
    err = MPU6050_WriteReg(0x19, 0x4F);
    if (err != FSP_SUCCESS) return err;
    // 0x1A=配置寄存器，0x03=DLPF=41Hz，关闭FIFO，开启陀螺仪+加速度计同步
    err = MPU6050_WriteReg(0x1A, 0x03);
    if (err != FSP_SUCCESS) return err;

    // 5. 开启加速度计+陀螺仪所有轴（0x6C=电源管理2寄存器，0x00=全轴使能）
    err = MPU6050_WriteReg(0x6C, 0x00);
    if (err != FSP_SUCCESS) return err;

    printf("MPU6050 高灵敏+高稳定配置完成！\r\n");
    return FSP_SUCCESS;
}
void MPU6050_Calibrate(void)
{
    uint16_t calib_cnt = 500; // 采样500次（约500ms，静态时执行）
    int32_t acc_raw[3] = {0};
    int32_t gyro_raw[3] = {0};
    uint8_t raw_data[14] = {0};

    printf("正在校准MPU6050零漂，请保持设备静止...\r\n");
    HAL_Delay(1000); // 等待设备稳定

    // 1. 采集静态原始数据
    for (uint16_t i = 0; i < calib_cnt; i++)
    {
        MPU6050_ReadRegs(raw_data, 0x3B, 14);
        // 累加加速度计原始值
        acc_raw[0] += (int16_t)(raw_data[0] << 8 | raw_data[1]);
        acc_raw[1] += (int16_t)(raw_data[2] << 8 | raw_data[3]);
        acc_raw[2] += (int16_t)(raw_data[4] << 8 | raw_data[5]);
        // 累加陀螺仪原始值
        gyro_raw[0] += (int16_t)(raw_data[8] << 8 | raw_data[9]);
        gyro_raw[1] += (int16_t)(raw_data[10] << 8 | raw_data[11]);
        gyro_raw[2] += (int16_t)(raw_data[12] << 8 | raw_data[13]);
        HAL_Delay(1); // 1ms采样一次，匹配SysTick频率
    }

    // 2. 计算零漂偏移（取平均值）
    Accel_Offset[0] = (float)acc_raw[0] / calib_cnt / 16384.0f; // 加速度计X轴偏移（g）
    Accel_Offset[1] = (float)acc_raw[1] / calib_cnt / 16384.0f; // Y轴偏移
    Accel_Offset[2] = (float)acc_raw[2] / calib_cnt / 16384.0f - 1.0f; // Z轴偏移（静态时Z轴应接近1g）
    Gyro_Offset[0] = (float)gyro_raw[0] / calib_cnt / 131.0f; // 陀螺仪X轴偏移（°/s）
    Gyro_Offset[1] = (float)gyro_raw[1] / calib_cnt / 131.0f; // Y轴偏移
    Gyro_Offset[2] = (float)gyro_raw[2] / calib_cnt / 131.0f; // Z轴偏移

    printf("MPU6050零漂校准完成！\r\n");
    printf("加速度计偏移：X=%.4fg Y=%.4fg Z=%.4fg\r\n", Accel_Offset[0], Accel_Offset[1], Accel_Offset[2]);
    printf("陀螺仪偏移：X=%.4f°/s Y=%.4f°/s Z=%.4f°/s\r\n", Gyro_Offset[0], Gyro_Offset[1], Gyro_Offset[2]);
}
/************************* 读取六轴+温度数据（基于现有ReadRegs，一次性读14字节） *************************/
fsp_err_t MPU6050_Read_SixAxis(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t raw_data[14] = {0}; // 14字节缓冲区：0x3B开始的连续寄存器

    // 一次性读取14字节（0x3B=加速度计X高字节，连续到陀螺仪Z低字节）
    err = MPU6050_ReadRegs(raw_data, 0x3B, 14);
    if (err != FSP_SUCCESS) return err;

    // 1. 拼接加速度计16位原始数据（高字节<<8 | 低字节）
    MPU6050_AccelX = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    MPU6050_AccelY = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    MPU6050_AccelZ = (int16_t)(raw_data[4] << 8 | raw_data[5]);

    // 2. 拼接温度数据（公式：Temp(℃) = (raw_temp/340.0) + 36.53）
    int16_t raw_temp = (int16_t)(raw_data[6] << 8 | raw_data[7]);
    MPU6050_Temp = (float)raw_temp / 340.0f + 36.53f;

    // 3. 拼接陀螺仪16位原始数据
    MPU6050_GyroX = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    MPU6050_GyroY = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    MPU6050_GyroZ = (int16_t)(raw_data[12] << 8 | raw_data[13]);

    // 4. 转换为物理值（基于±2g/±250°/s量程的分辨率）
// 修复后（添加零漂补偿）
AccelX_g = (float)MPU6050_AccelX / 16384.0f - Accel_Offset[0];
AccelY_g = (float)MPU6050_AccelY / 16384.0f - Accel_Offset[1];
AccelZ_g = (float)MPU6050_AccelZ / 16384.0f - Accel_Offset[2];
GyroX_dps = (float)MPU6050_GyroX / 131.0f - Gyro_Offset[0];
GyroY_dps = (float)MPU6050_GyroY / 131.0f - Gyro_Offset[1];
GyroZ_dps = (float)MPU6050_GyroZ / 131.0f - Gyro_Offset[2];

    return FSP_SUCCESS;
}

/************************* 互补滤波姿态解算（反应灵敏+静态稳定，无漂移） *************************/
void MPU6050_AHRS_Complementary(void)
{
    // 1. 计算两次解算的时间差dt（单位：s，SysTick_Handler中实现ms计时）
    uint32_t now_time = HAL_GetTick(); // 替换为你的系统ms级计时函数（必做！）
    float dt = (float)(now_time - last_time) / 1000.0f;
    last_time = now_time;
    if (dt <= 0.001f) dt = 0.001f; // 避免dt为0，防止除零错误
	
    // 计算加速度计的动态变化率（判断是否处于静态）
    float acc_mag = (float)sqrt(AccelX_g*AccelX_g + AccelY_g*AccelY_g + AccelZ_g*AccelZ_g);
    // 静态判断：加速度计变化率<0.05g（可根据实际调整）
    if (fabs(acc_mag - 1.0f) < 0.05f)
    {
        COMPLEMENTARY_COEFF = 0.90f; // 静态时：90%陀螺仪+10%加速度计，增大加速度计权重，增强校准
    }
    else
    {
        COMPLEMENTARY_COEFF = 0.98f; // 动态时：98%陀螺仪+2%加速度计，保持灵敏性
    }
    // 2. 加速度计解算静态横滚角/俯仰角（利用重力加速度，无漂移）
		
    if (acc_mag < 0.8f || acc_mag > 1.2f) return; // 加速度过大/过小，放弃校准（运动中）
    float roll_acc = (float)atan2(AccelY_g, AccelZ_g) * 180.0f / M_PI;
    float pitch_acc = (float)atan2(-AccelX_g, (float)sqrt(AccelY_g*AccelY_g + AccelZ_g*AccelZ_g)) * 180.0f / M_PI;// 俯仰角（°）

    // 3. 陀螺仪解算动态角度增量（角速度积分，快速响应）
    float roll_gyro = Roll + GyroX_dps * dt;   // 横滚角=上一次角度+角速度×时间
    float pitch_gyro = Pitch + GyroY_dps * dt; // 俯仰角=上一次角度+角速度×时间
    Yaw += GyroZ_dps * dt;                     // 偏航角（无磁力计，会缓慢漂移，仅作参考）

    // 4. 互补滤波融合：98%陀螺仪（动态） + 2%加速度计（静态校准）
    Roll = COMPLEMENTARY_COEFF * roll_gyro + (1 - COMPLEMENTARY_COEFF) * roll_acc;
    Pitch = COMPLEMENTARY_COEFF * pitch_gyro + (1 - COMPLEMENTARY_COEFF) * pitch_acc;

    // 角度限幅（-180°~180°，可选）
    if (Roll > 180.0f) Roll -= 360.0f;
    if (Roll < -180.0f) Roll += 360.0f;
    if (Pitch > 180.0f) Pitch -= 360.0f;
    if (Pitch < -180.0f) Pitch += 360.0f;
}
// 在MPU6050初始化+零漂校准后执行
void MPU6050_Init_Attitude(void)
{
    uint16_t init_cnt = 100;
    float roll_avg = 0.0f, pitch_avg = 0.0f;
    //uint8_t raw_data[14] = {0};

    printf("正在校准初始姿态，请保持设备静止...\r\n");
    HAL_Delay(1000);

    for (uint16_t i = 0; i < init_cnt; i++)
    {
        MPU6050_Read_SixAxis();
        float acc_mag = (float)sqrt(AccelX_g*AccelX_g + AccelY_g*AccelY_g + AccelZ_g*AccelZ_g);
        if (acc_mag >= 0.8f && acc_mag <= 1.2f)
        {
            roll_avg += (float)atan2(AccelY_g, AccelZ_g) * 180.0f / M_PI;
						pitch_avg += (float)atan2(-AccelX_g, (float)sqrt(AccelY_g*AccelY_g + AccelZ_g*AccelZ_g)) * 180.0f / M_PI;
        }
        HAL_Delay(1);
    }

    // 初始姿态赋值
    Roll = roll_avg / init_cnt;
    Pitch = pitch_avg / init_cnt;
    Yaw = 0.0f;
    printf("初始姿态校准完成！Roll=%.1f° Pitch=%.1f°\r\n", Roll, Pitch);
}

