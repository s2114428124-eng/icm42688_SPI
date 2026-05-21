#include "icm42688.h"

/* ICM-42688-P I2C 地址 (AD0接地为0x68，接高电平为0x69) */
#define ICM42688P_ADDR         0x68
#define ICM42688P_I2C_ADDR_W   (ICM42688P_ADDR << 1)
#define ICM42688P_I2C_ADDR_R   ((ICM42688P_ADDR << 1) | 0x01)

/* 常用寄存器地址 (Bank 0) */
#define REG_DEVICE_CONFIG      0x11
#define REG_PWR_MGMT0          0x4E
#define REG_GYRO_CONFIG0       0x4F
#define REG_ACCEL_CONFIG0      0x50
#define REG_TEMP_DATA1         0x1D
#define REG_WHO_AM_I           0x75
#define REG_BANK_SEL 	   			 0x76
#define ICM42688P_WHO_AM_I_VAL 0x47
#define M_PI 3.1415926535f
// 全局数据实例
ICM42688P_Data_t icm_data = {0};
float Offset_gyro[7] = {0,0,0,0,0,0,0};
// ================== 1. 软件 I2C 底层实现 ==================

static void I2C_Delay(void) {
    __IO uint32_t i = 150;
    while(i--);
}

static void I2C_Start(void) {
    I2C_SDA_H(); I2C_SCL_H(); I2C_Delay();
    I2C_SDA_L(); I2C_Delay(); I2C_SCL_L();
}

static void I2C_Stop(void) {
    I2C_SDA_L(); I2C_SCL_L(); I2C_Delay();
    I2C_SCL_H(); I2C_Delay(); I2C_SDA_H(); I2C_Delay();
}

static uint8_t I2C_WaitAck(void) {
    uint8_t ack = 0;
    I2C_SDA_H(); I2C_Delay(); I2C_SCL_H(); I2C_Delay();
    if (I2C_SDA_READ() == GPIO_PIN_SET) ack = 1;
    I2C_SCL_L(); return ack;
}

static void I2C_Ack(void)  { I2C_SCL_L(); I2C_SDA_L(); I2C_Delay(); I2C_SCL_H(); I2C_Delay(); I2C_SCL_L(); }
static void I2C_NAck(void) { I2C_SCL_L(); I2C_SDA_H(); I2C_Delay(); I2C_SCL_H(); I2C_Delay(); I2C_SCL_L(); }

static void I2C_SendByte(uint8_t byte) {
    for (uint8_t i = 0; i < 8; i++) {
        if (byte & 0x80) I2C_SDA_H(); else I2C_SDA_L();
        byte <<= 1; I2C_Delay(); I2C_SCL_H(); I2C_Delay(); I2C_SCL_L();
    }
}

static uint8_t I2C_ReadByte(uint8_t ack) {
    uint8_t byte = 0;
    I2C_SDA_H();
    for (uint8_t i = 0; i < 8; i++) {
        byte <<= 1; I2C_SCL_H(); I2C_Delay();
        if (I2C_SDA_READ() == GPIO_PIN_SET) byte |= 0x01;
        I2C_SCL_L(); I2C_Delay();
    }
    if (ack) I2C_Ack(); else I2C_NAck();
    return byte;
}

// ================== 2. ICM-42688-P 寄存器读写接口 ==================

/**
 * @brief 向ICM42688P指定寄存器写入1字节数据
 */
static uint8_t ICM_WriteReg(uint8_t reg, uint8_t data) {
    I2C_Start();
    I2C_SendByte(ICM42688P_I2C_ADDR_W);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    I2C_SendByte(reg);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    I2C_SendByte(data);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    I2C_Stop();
    return 0;
}

/**
 * @brief 从ICM42688P指定寄存器连续读取多字节数据
 */
static uint8_t ICM_ReadRegs(uint8_t reg, uint8_t *pData, uint16_t len) {
    I2C_Start();
    I2C_SendByte(ICM42688P_I2C_ADDR_W);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    I2C_SendByte(reg);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    I2C_Start(); // 重复起始条件
    I2C_SendByte(ICM42688P_I2C_ADDR_R);
    if (I2C_WaitAck()) { I2C_Stop(); return 1; }

    for (uint16_t i = 0; i < len; i++) {
        pData[i] = I2C_ReadByte((i == len - 1) ? 0 : 1);
    }

    I2C_Stop();
    return 0;
}

// ================== 3. 用户核心调用接口 ==================

/**
 * @brief 初始化 ICM-42688-P
 * @retval 0:成功, 1:失败
 */
uint8_t ICM42688P_Init(void) {
    uint8_t who_am_i = 0;
    uint8_t pwr_check = 0;

    I2C_SDA_H();
    I2C_SCL_H();
    HAL_Delay(10);

    // 1. 验证 WHO_AM_I
    if (ICM_ReadRegs(REG_WHO_AM_I, &who_am_i, 1) != 0) return 1;
    if (who_am_i != ICM42688P_WHO_AM_I_VAL) return 2;

    // 2. 软复位 (DEVICE_CONFIG = 0x01)
    ICM_WriteReg(REG_DEVICE_CONFIG, 0x01);
    HAL_Delay(50);  // 手册要求 ≥30ms

    // 3. 开启低噪声模式，并验证写入成功
    for (int retry = 0; retry < 5; retry++) {
        ICM_WriteReg(REG_PWR_MGMT0, 0x0F);
        HAL_Delay(2);
        ICM_ReadRegs(REG_PWR_MGMT0, &pwr_check, 1);
        if (pwr_check == 0x0F) break;
    }
    if (pwr_check != 0x0F) return 3;

    // 4. 配置加速度计: 量程 ±16g, ODR 1kHz
    ICM_WriteReg(REG_ACCEL_CONFIG0, 0x06);
    // 5. 配置陀螺仪: 量程 ±2000dps, ODR 1kHz
    ICM_WriteReg(REG_GYRO_CONFIG0, 0x06);

    // 6. UI 滤波器 (陀螺仪16Hz二阶, 加速度计16Hz二阶)
    ICM_WriteReg(0x51, 0x30);
    ICM_WriteReg(0x52, 0x30);

    // 7. 去抖滤波器 (官方推荐)
    ICM_WriteReg(0x5F, 0x04);

    uint8_t reg_val[5];
    ICM_ReadRegs(REG_PWR_MGMT0,     &reg_val[0], 1);
    ICM_ReadRegs(REG_GYRO_CONFIG0,  &reg_val[1], 1);
    ICM_ReadRegs(REG_ACCEL_CONFIG0, &reg_val[2], 1);
    ICM_ReadRegs(0x51,              &reg_val[3], 1);
    ICM_ReadRegs(0x52,              &reg_val[4], 1);
    //printf("Regs: PWR=0x%02X GYRO0=0x%02X ACC0=0x%02X GYRO1=0x%02X ACC1=0x%02X\r\n",
           //reg_val[0], reg_val[1], reg_val[2], reg_val[3], reg_val[4]);
    HAL_Delay(2000);

    // 12. 零偏校准
    ICM42688P_Offset();

    return 0;
}
/**
 * @brief 在主循环或定时器中周期调用的数据读取任务
 */
void ICM42688P_Task(void) {
    uint8_t raw_data[14];
//printf("%02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\r\n",
//       raw_data[0], raw_data[1], raw_data[2], raw_data[3],
//       raw_data[4], raw_data[5], raw_data[6], raw_data[7],
//       raw_data[8], raw_data[9], raw_data[10], raw_data[11],
//       raw_data[12], raw_data[13]);
    // 从 TEMP_DATA1 寄存器开始，连续读取 14 个字节 (温度2 + 加速度计6 + 陀螺仪6)
    if (ICM_ReadRegs(REG_TEMP_DATA1, raw_data, 14) != 0) return;

    // 1. 合并高低字节 (大端模式)
    icm_data.temp_raw    = (int16_t)((raw_data[0] << 8) | raw_data[1]);
    icm_data.accel_raw_x = (int16_t)((raw_data[2] << 8) | raw_data[3]);
    icm_data.accel_raw_y = (int16_t)((raw_data[4] << 8) | raw_data[5]);
    icm_data.accel_raw_z = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    icm_data.gyro_raw_x  = (int16_t)((raw_data[8] << 8) | raw_data[9]);
    icm_data.gyro_raw_y  = (int16_t)((raw_data[10] << 8) | raw_data[11]);
    icm_data.gyro_raw_z  = (int16_t)((raw_data[12] << 8) | raw_data[13]);

//    // 2. 转换为物理量
//    // 加速度灵敏度: ±16g -> 32768 / 16 = 2048 LSB/g
    icm_data.accel_x = (float)icm_data.accel_raw_x / 2048.0f;
    icm_data.accel_y = (float)icm_data.accel_raw_y / 2048.0f;
    icm_data.accel_z = (float)icm_data.accel_raw_z / 2048.0f;

    icm_data.gyro_x = ((float)icm_data.gyro_raw_x / 16.384f) * (M_PI / 180.0f);
    icm_data.gyro_y = ((float)icm_data.gyro_raw_y / 16.384f) * (M_PI / 180.0f);
    icm_data.gyro_z = ((float)icm_data.gyro_raw_z / 16.384f) * (M_PI / 180.0f);
		if(Offset_gyro[3]){
		icm_data.gyro_x-=Offset_gyro[0];
		icm_data.gyro_y-=Offset_gyro[1];
		icm_data.gyro_z-=Offset_gyro[2];
		icm_data.accel_x -=Offset_gyro[4];
		icm_data.accel_y -=Offset_gyro[5];
		icm_data.accel_z -=Offset_gyro[6];			
		}
			
    // 陀螺仪灵敏度: 去漂移
		//if(icm_data.gyro_x<0.011&&icm_data.gyro_x>-0.011)icm_data.gyro_x=0;
		//if(icm_data.gyro_y<0.011&&icm_data.gyro_y>-0.011)icm_data.gyro_y =0;
		//if(icm_data.gyro_z<0.011&&icm_data.gyro_z>-0.011)icm_data.gyro_z =0;
    // 温度转换公式 (根据官方数据手册): (Temp_raw / 132.48) + 25
    icm_data.temp = ((float)icm_data.temp_raw / 132.48f) + 25.0f;
    icm_data.data_updated = 1;
}

//	icm_data.accel_x / y / z   g (重力加速度)
//  icm_data.gyro_x / y / z    弧度/秒 (rad/s)
void ICM42688P_Offset(void) {
    float sum_gx = 0, sum_gy = 0, sum_gz = 0;
    float sum_accx = 0, sum_accy = 0, sum_accz = 0; 
    const int samples = 1000;         // 采集样本数
    
    Offset_gyro[3] = 0; 
    
    // 等待传感器稳定（很重要）
    HAL_Delay(100);
    
    // 丢弃前几个读数（让滤波器稳定）
    for (int i = 0; i < 10; i++) {
        ICM42688P_Task();
        HAL_Delay(5);
    }
    
    // 正式采集
    for (int i = 0; i < samples; i++) {
        ICM42688P_Task();
        sum_gx += icm_data.gyro_x;
        sum_gy += icm_data.gyro_y;
        sum_gz += icm_data.gyro_z;
        sum_accx += icm_data.accel_x;
        sum_accy += icm_data.accel_y;
        sum_accz += icm_data.accel_z;
        HAL_Delay(5);   // 【建议调整】因为你上面算的是2ms一次，这里设为2即可
    }
    
    // 计算陀螺仪零偏 (理论值为0)
    Offset_gyro[0] = sum_gx / samples;   
    Offset_gyro[1] = sum_gy / samples;
    Offset_gyro[2] = sum_gz / samples;
    
    // 计算加速度计水平零偏 (理论值为0)
    Offset_gyro[4] = sum_accx / samples;
    Offset_gyro[5] = sum_accy / samples;
    
    // 【修复1】保留Z轴重力加速度。假设校准时板子正面朝上，Z轴理论值为 +1.0g
    // 如果你的板子校准时是倒着放的，这里应该是 + 1.0f
    Offset_gyro[6] = (sum_accz / samples) - 1.0f; 
    
    // 标记校准完成，开启补偿
    Offset_gyro[3] = 1;                  
}