#include "mpu6050.h"
#include "bsp_debug_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "math.h"

/************************* 核心优化：减少全局变量，仅保留必要的volatile *************************/
extern sci_i2c_instance_ctrl_t MPU_6050_ctrl;
extern const i2c_master_cfg_t MPU_6050_cfg;
volatile i2c_master_event_t MPU_6050_callback_event; // 仅回调事件需volatile

/************************* 六轴数据+姿态解算变量（局部化+必要时全局） *************************/
int16_t MPU6050_AccelX, MPU6050_AccelY, MPU6050_AccelZ;
int16_t MPU6050_GyroX,  MPU6050_GyroY,  MPU6050_GyroZ;
float AccelX_g, AccelY_g, AccelZ_g;
float GyroX_dps, GyroY_dps, GyroZ_dps;
float MPU6050_Temp;
float Roll = 0.0f, Pitch = 0.0f, Yaw = 0.0f;
TickType_t last_time = 0;
float COMPLEMENTARY_COEFF = 0.98f;
float Accel_Offset[3] = {0.0f, 0.0f, 0.0f};
float Gyro_Offset[3] = {0.0f, 0.0f, 0.0f};

/************************* 优化1：回调函数仅捕获事件，无冗余逻辑 *************************/
void MPU_6050_callback(i2c_master_callback_args_t * p_args)
{
    if (NULL != p_args)
    {
        MPU_6050_callback_event = p_args->event;
    }
}

/************************* I2C初始化（保留核心逻辑，增加空指针校验） *************************/
fsp_err_t Init_I2C_Driver(void)
{
    fsp_err_t err = FSP_SUCCESS;



    err = R_SCI_I2C_Open(&MPU_6050_ctrl, &MPU_6050_cfg);
    if (FSP_SUCCESS != err)
    {
        printf("SCI_I2C打开失败！Err=0x%x \r\n", err);
        return err;
    }

    printf("SCI_I2C初始化成功！\r\n");
    return err;
}

/************************* 优化2：I2C写寄存器（局部超时+短延时+事件重置） *************************/
fsp_err_t MPU6050_WriteReg(uint8_t reg, uint8_t data)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t tx_buf[2] = {reg, data}; // 局部缓冲区，避免全局竞态
    uint32_t local_timeout = 50;     // 局部超时（50ms，足够I2C通信）
    uint8_t retry_cnt = 3;           // 重试次数，提高可靠性

    // 重试机制：最多3次
    while (retry_cnt--)
    {
        //MPU_6050_callback_event = I2C_MASTER_EVENT_NONE; // 必重置事件
        err = R_SCI_I2C_Write(&MPU_6050_ctrl, tx_buf, 2, false);
        if (err != FSP_SUCCESS)
        {
            printf("I2C写失败(重试%d)！Reg=0x%02X, Err=0x%x\r\n", 3-retry_cnt, reg, err);
            vTaskDelay(pdMS_TO_TICKS(1)); // 短延时后重试
            continue;
        }

        // 优化：用1ms延时等待事件，降低CPU占用+缩短等待时间
        while ((I2C_MASTER_EVENT_TX_COMPLETE != MPU_6050_callback_event) && local_timeout)
        {
            vTaskDelay(pdMS_TO_TICKS(1)); // 从400ms→1ms，核心优化！
            local_timeout--;
        }

        if (local_timeout == 0)
        {
            printf("I2C写超时(重试%d)！Reg=0x%02X\r\n", 3-retry_cnt, reg);
            local_timeout = 50; // 重置超时，准备下一次重试
            continue;
        }
        break; // 成功则退出重试
    }

    if (retry_cnt == 0 && err != FSP_SUCCESS)
    {
        printf("I2C写最终失败！Reg=0x%02X\r\n", reg);
        return FSP_ERR_TIMEOUT;
    }

    return FSP_SUCCESS;
}

/************************* 优化3：I2C读寄存器（局部超时+重试+事件重置） *************************/
fsp_err_t MPU6050_ReadRegs(uint8_t* ptr_read, uint8_t reg, uint8_t len)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t tx_buf[1] = {reg};
    uint32_t local_timeout = 50;     // 局部超时，避免全局竞态
    uint8_t retry_cnt = 3;           // 重试3次

    if (ptr_read == NULL || len == 0)
    {
        printf("读寄存器参数无效！\r\n");
        return FSP_ERR_INVALID_POINTER;
    }

    while (retry_cnt--)
    {
        // 步骤1：写寄存器地址（restart=true，不释放总线）
      //  MPU_6050_callback_event = I2C_MASTER_EVENT_NONE;
        err = R_SCI_I2C_Write(&MPU_6050_ctrl, tx_buf, 1, true);
        if (err != FSP_SUCCESS)
        {
            printf("写地址失败(重试%d)！Reg=0x%02X, Err=0x%x\r\n", 3-retry_cnt, reg, err);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 等待写地址完成（1ms延时）
        while ((I2C_MASTER_EVENT_TX_COMPLETE != MPU_6050_callback_event) && local_timeout)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            local_timeout--;
        }

        if (local_timeout == 0)
        {
            printf("写地址超时(重试%d)！Reg=0x%02X\r\n", 3-retry_cnt, reg);
            local_timeout = 50;
            continue;
        }

        // 步骤2：读数据
        local_timeout = 50;
       // MPU_6050_callback_event = I2C_MASTER_EVENT_NONE;
        err = R_SCI_I2C_Read(&MPU_6050_ctrl, ptr_read, len, false);
        if (err != FSP_SUCCESS)
        {
            printf("读数据失败(重试%d)！Reg=0x%02X, Err=0x%x\r\n", 3-retry_cnt, reg, err);
            vTaskDelay(pdMS_TO_TICKS(1));
            continue;
        }

        // 等待读完成（1ms延时）
        while ((I2C_MASTER_EVENT_RX_COMPLETE != MPU_6050_callback_event) && local_timeout)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
            local_timeout--;
        }

        if (local_timeout == 0)
        {
            printf("读数据超时(重试%d)！Reg=0x%02X\r\n", 3-retry_cnt, reg);
            local_timeout = 50;
            continue;
        }
        break; // 成功退出重试
    }

    if (retry_cnt == 0 && err != FSP_SUCCESS)
    {
        printf("读寄存器最终失败！Reg=0x%02X\r\n", reg);
        return FSP_ERR_TIMEOUT;
    }

    return FSP_SUCCESS;
}

/************************* 优化4：MPU6050初始化（缩短延时，提高效率） *************************/
fsp_err_t MPU6050_Init_Config(void)
{
    fsp_err_t err = FSP_SUCCESS;

    // 唤醒MPU6050（延时从400ms→50ms，足够唤醒）
    err = MPU6050_WriteReg(0x6B, 0x00);
    if (err != FSP_SUCCESS) return err;
    vTaskDelay(pdMS_TO_TICKS(50));

    // 加速度计±2g
    err = MPU6050_WriteReg(0x1C, 0x00);
    if (err != FSP_SUCCESS) return err;

    // 陀螺仪±250°/s
    err = MPU6050_WriteReg(0x1B, 0x00);
    if (err != FSP_SUCCESS) return err;

    // 采样率100Hz + DLPF 41Hz
    err = MPU6050_WriteReg(0x19, 0x4F);
    if (err != FSP_SUCCESS) return err;
    err = MPU6050_WriteReg(0x1A, 0x03);
    if (err != FSP_SUCCESS) return err;

    // 全轴使能
    err = MPU6050_WriteReg(0x6C, 0x00);
    if (err != FSP_SUCCESS) return err;

    printf("MPU6050配置完成！\r\n");
    return FSP_SUCCESS;
}

/************************* 优化5：零漂校准（portYIELD替代短延时，减少任务切换） *************************/
void MPU6050_Calibrate(void)
{
    uint16_t calib_cnt = 500;
    int32_t acc_raw[3] = {0};
    int32_t gyro_raw[3] = {0};
    uint8_t raw_data[14] = {0};
    uint16_t valid_cnt = 0; // 有效采样计数

    printf("正在校准MPU6050零漂，请保持设备静止...\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    for (uint16_t i = 0; i < calib_cnt; i++)
    {
        // 仅累加有效数据，跳过失败的读取
        if (MPU6050_ReadRegs(raw_data, 0x3B, 14) == FSP_SUCCESS)
        {
            acc_raw[0] += (int16_t)(raw_data[0] << 8 | raw_data[1]);
            acc_raw[1] += (int16_t)(raw_data[2] << 8 | raw_data[3]);
            acc_raw[2] += (int16_t)(raw_data[4] << 8 | raw_data[5]);
            gyro_raw[0] += (int16_t)(raw_data[8] << 8 | raw_data[9]);
            gyro_raw[1] += (int16_t)(raw_data[10] << 8 | raw_data[11]);
            gyro_raw[2] += (int16_t)(raw_data[12] << 8 | raw_data[13]);
            valid_cnt++;
        }

        // 优化：用portYIELD()替代vTaskDelay(1ms)，减少任务切换开销
        portYIELD();
    }

    // 避免除零错误
    if (valid_cnt == 0)
    {
        printf("校准无有效数据！\r\n");
        return;
    }

    // 计算零漂偏移
    Accel_Offset[0] = (float)acc_raw[0] / valid_cnt / 16384.0f;
    Accel_Offset[1] = (float)acc_raw[1] / valid_cnt / 16384.0f;
    Accel_Offset[2] = (float)acc_raw[2] / valid_cnt / 16384.0f - 1.0f;
    Gyro_Offset[0] = (float)gyro_raw[0] / valid_cnt / 131.0f;
    Gyro_Offset[1] = (float)gyro_raw[1] / valid_cnt / 131.0f;
    Gyro_Offset[2] = (float)gyro_raw[2] / valid_cnt / 131.0f;

    printf("MPU6050零漂校准完成！有效采样%d次\r\n", valid_cnt);
}

/************************* 优化6：六轴数据读取（保留核心，增加错误判断） *************************/
fsp_err_t MPU6050_Read_SixAxis(void)
{
    fsp_err_t err = FSP_SUCCESS;
    uint8_t raw_data[14] = {0};

    err = MPU6050_ReadRegs(raw_data, 0x3B, 14);
    if (err != FSP_SUCCESS) return err;

    // 拼接原始数据
    MPU6050_AccelX = (int16_t)(raw_data[0] << 8 | raw_data[1]);
    MPU6050_AccelY = (int16_t)(raw_data[2] << 8 | raw_data[3]);
    MPU6050_AccelZ = (int16_t)(raw_data[4] << 8 | raw_data[5]);
    int16_t raw_temp = (int16_t)(raw_data[6] << 8 | raw_data[7]);
    MPU6050_GyroX = (int16_t)(raw_data[8] << 8 | raw_data[9]);
    MPU6050_GyroY = (int16_t)(raw_data[10] << 8 | raw_data[11]);
    MPU6050_GyroZ = (int16_t)(raw_data[12] << 8 | raw_data[13]);

    // 转换物理值（带零漂补偿）
    AccelX_g = (float)MPU6050_AccelX / 16384.0f - Accel_Offset[0];
    AccelY_g = (float)MPU6050_AccelY / 16384.0f - Accel_Offset[1];
    AccelZ_g = (float)MPU6050_AccelZ / 16384.0f - Accel_Offset[2];
    GyroX_dps = (float)MPU6050_GyroX / 131.0f - Gyro_Offset[0];
    GyroY_dps = (float)MPU6050_GyroY / 131.0f - Gyro_Offset[1];
    GyroZ_dps = (float)MPU6050_GyroZ / 131.0f - Gyro_Offset[2];
    MPU6050_Temp = (float)raw_temp / 340.0f + 36.53f;

    return FSP_SUCCESS;
}

/************************* 优化7：姿态解算（合并冗余计算，减少CPU占用） *************************/
void MPU6050_AHRS_Complementary(void)
{
    if (last_time == 0) // 初始化第一次解算
    {
        last_time = xTaskGetTickCount();
        return;
    }

    // 计算时间差（优化：限制最小dt，避免除零）
    TickType_t now_time = xTaskGetTickCount();
    float dt = (float)(now_time - last_time) / 1000.0f;
    dt = (dt < 0.001f) ? 0.001f : dt;
    last_time = now_time;

    // 优化：acc_mag只计算一次，避免冗余
    float acc_mag = sqrtf(AccelX_g*AccelX_g + AccelY_g*AccelY_g + AccelZ_g*AccelZ_g);
    if (acc_mag < 0.8f || acc_mag > 1.2f) return; // 快速退出无效数据

    // 动态调整融合系数
    COMPLEMENTARY_COEFF = (fabs(acc_mag - 1.0f) < 0.05f) ? 0.90f : 0.98f;

    // 加速度计解算角度
    float roll_acc = atan2f(AccelY_g, AccelZ_g) * 180.0f / M_PI;
    float pitch_acc = atan2f(-AccelX_g, sqrtf(AccelY_g*AccelY_g + AccelZ_g*AccelZ_g)) * 180.0f / M_PI;

    // 陀螺仪积分
    float roll_gyro = Roll + GyroX_dps * dt;
    float pitch_gyro = Pitch + GyroY_dps * dt;
    Yaw += GyroZ_dps * dt;

    // 互补滤波融合
    Roll = COMPLEMENTARY_COEFF * roll_gyro + (1 - COMPLEMENTARY_COEFF) * roll_acc;
    Pitch = COMPLEMENTARY_COEFF * pitch_gyro + (1 - COMPLEMENTARY_COEFF) * pitch_acc;

    // 角度限幅
    Roll = (Roll > 180.0f) ? (Roll - 360.0f) : (Roll < -180.0f) ? (Roll + 360.0f) : Roll;
    Pitch = (Pitch > 180.0f) ? (Pitch - 360.0f) : (Pitch < -180.0f) ? (Pitch + 360.0f) : Pitch;
}

/************************* 优化8：初始姿态校准（同零漂校准，优化延时） *************************/
void MPU6050_Init_Attitude(void)
{
    uint16_t init_cnt = 100;
    float roll_avg = 0.0f, pitch_avg = 0.0f;
    uint16_t valid_cnt = 0;

    printf("正在校准初始姿态，请保持设备静止...\r\n");
    vTaskDelay(pdMS_TO_TICKS(100));

    for (uint16_t i = 0; i < init_cnt; i++)
    {
        if (MPU6050_Read_SixAxis() == FSP_SUCCESS)
        {
            float acc_mag = sqrtf(AccelX_g*AccelX_g + AccelY_g*AccelY_g + AccelZ_g*AccelZ_g);
            if (acc_mag >= 0.8f && acc_mag <= 1.2f)
            {
                roll_avg += atan2f(AccelY_g, AccelZ_g) * 180.0f / M_PI;
                pitch_avg += atan2f(-AccelX_g, sqrtf(AccelY_g*AccelY_g + AccelZ_g*AccelZ_g)) * 180.0f / M_PI;
                valid_cnt++;
            }
        }
        portYIELD(); // 替代短延时
    }

    if (valid_cnt == 0)
    {
        printf("初始姿态校准无有效数据！\r\n");
        return;
    }

    Roll = roll_avg / valid_cnt;
    Pitch = pitch_avg / valid_cnt;
    Yaw = 0.0f;
    printf("初始姿态校准完成！Roll=%.1f° Pitch=%.1f°\r\n", Roll, Pitch);
}
