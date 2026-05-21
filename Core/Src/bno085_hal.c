#include "bno085_hal.h"
#include <math.h>

#define BNO080_DEFAULT_ADDRESS 0x4B
#define BNO085_I2C_ADDR_W  0x96  // (0x4B << 1)
#define BNO085_I2C_ADDR_R  0x97
#define SHTP_HEADER_SIZE   4

// 全局数据实例
BNO085_Data_t bno085_data = {0};

// SHTP 接收缓冲区
static uint8_t shtp_buf[256];

// ================== 1. 软件 I2C 底层实现 ==================

static void I2C_Delay(void) {
    __IO uint32_t i = 150; // H750 粗略延时，实际项目建议换成 delay_us(2);
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

// ================== 2. BNO085 接口函数 ==================

static uint8_t BNO085_I2C_Write(uint8_t *pData, uint16_t len) {
    I2C_Start();
    I2C_SendByte(BNO085_I2C_ADDR_W);
    if (I2C_WaitAck() != 0) { I2C_Stop(); return 1; }
    for (uint16_t i = 0; i < len; i++) {
        I2C_SendByte(pData[i]);
        if (I2C_WaitAck() != 0) { I2C_Stop(); return 2; }
    }
    I2C_Stop(); return 0;
}

static uint8_t BNO085_I2C_Read(uint8_t *pData, uint16_t len) {
    if (len == 0) return 0;
    I2C_Start();
    I2C_SendByte(BNO085_I2C_ADDR_R);
    if (I2C_WaitAck() != 0) { I2C_Stop(); return 1; }
    for (uint16_t i = 0; i < len; i++) {
        pData[i] = I2C_ReadByte((i == len - 1) ? 0 : 1);
    }
    I2C_Stop(); return 0;
}

static uint8_t BNO085_WaitForData(uint32_t timeout) {
    uint32_t start = HAL_GetTick();
    while (HAL_GPIO_ReadPin(BNO085_INT_PORT, BNO085_INT_PIN) == GPIO_PIN_SET) {
        if (HAL_GetTick() - start > timeout) return 1;
    }
    return 0;
}

// ================== 3. SHTP 协议与数据解析 ==================

static void BNO085_ParseSensorReport(uint8_t *payload, uint16_t length) {
    uint16_t index = 0;
    while (index < length) {
        uint8_t report_id = payload[index];
        
        if (report_id == 0x08) { // 游戏旋转矢量
            if (index + 14 <= length) {
                bno085_data.quat_accuracy = payload[index + 2] & 0x03;
                
                int16_t raw_i = (payload[index + 5] << 8) | payload[index + 4];
                int16_t raw_j = (payload[index + 7] << 8) | payload[index + 6];
                int16_t raw_k = (payload[index + 9] << 8) | payload[index + 8];
                int16_t raw_r = (payload[index + 11] << 8) | payload[index + 10];

                bno085_data.quat_x = (float)raw_i / 16384.0f;
                bno085_data.quat_y = (float)raw_j / 16384.0f;
                bno085_data.quat_z = (float)raw_k / 16384.0f;
                bno085_data.quat_w = (float)raw_r / 16384.0f;

                // 计算欧拉角 (度)
                float sqw = bno085_data.quat_w * bno085_data.quat_w;
                float sqx = bno085_data.quat_x * bno085_data.quat_x;
                float sqy = bno085_data.quat_y * bno085_data.quat_y;
                float sqz = bno085_data.quat_z * bno085_data.quat_z;
                
                bno085_data.pitch = asinf(2.0f * (bno085_data.quat_w * bno085_data.quat_y - bno085_data.quat_z * bno085_data.quat_x)) * 180.0f / 3.14159265f;
                bno085_data.roll  = atan2f(2.0f * (bno085_data.quat_w * bno085_data.quat_x + bno085_data.quat_y * bno085_data.quat_z), sqw - sqx - sqy + sqz) * 180.0f / 3.14159265f;
                bno085_data.yaw   = atan2f(2.0f * (bno085_data.quat_w * bno085_data.quat_z + bno085_data.quat_x * bno085_data.quat_y), sqw + sqx - sqy - sqz) * 180.0f / 3.14159265f;
                
                bno085_data.data_updated = 1;
            }
            index += 14;
        } 
        else if (report_id == 0xFB) { // 基础时间参考
            index += 5;
        }
        else {
            break; // 遇到未处理的ID直接退出，防止死循环
        }
    }
}

// ================== 4. 用户核心调用接口 ==================

/**
 * @brief 初始化 BNO085 并激活游戏旋转矢量输出
 */
uint8_t BNO085_Init(void) {
    I2C_SDA_H();
    I2C_SCL_H();
    
    // 1. 硬件复位
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_RESET);
    HAL_Delay(10); 
    HAL_GPIO_WritePin(BNO085_RST_PORT, BNO085_RST_PIN, GPIO_PIN_SET);
    
    // 2. 等待启动完成 (INT引脚会拉低一次表示就绪，或直接死等)
    HAL_Delay(300); 

    // 3. 构造 "Set Feature Command" 以激活 Game Rotation Vector (0x08)
    // 设置刷新率为 100Hz (10000us -> 0x00002710)
    uint8_t cmd_enable_grv[] = {
        21, 0, 2, 0,            // SHTP Header: 包长21, 通道2(控制), Seq0
        0xFD, 0x08, 0x00, 0x00, // Report ID(FD), Feature(08), Flags(00)
        0x00, 0x10, 0x27, 0x00, // 刷新率 LSB...
        0x00, 0x00, 0x00, 0x00, // Batch Interval
        0x00, 0x00, 0x00, 0x00, // Sensor Config
        0x00
    };

    // 4. 发送唤醒激活命令
    if (BNO085_I2C_Write(cmd_enable_grv, sizeof(cmd_enable_grv)) != 0) {
        return 1; // 初始化失败（I2C通信异常）
    }
    
    return 0; // 成功
}

/**
 * @brief 在主循环中周期调用的处理任务
 */
void BNO085_Task(void) {
    // 1. 如果 INT 没有拉低，直接返回，不阻塞 CPU
    if (BNO085_WaitForData(0) != 0) return; 

    // 2. 读取 4 字节 Header
    if (BNO085_I2C_Read(shtp_buf, SHTP_HEADER_SIZE) != 0) return;

    // 3. 解析包长度
    uint16_t packet_length = ((uint16_t)shtp_buf[1] << 8 | shtp_buf[0]) & 0x7FFF;

    // 4. 长度合法性校验
    if (packet_length <= SHTP_HEADER_SIZE || packet_length > sizeof(shtp_buf)) {
        // 清空总线残留数据
        BNO085_I2C_Read(shtp_buf, 10); 
        return; 
    }

    // 5. 连续读取剩余的有效载荷 Payload
    if (BNO085_I2C_Read(&shtp_buf[4], packet_length - 4) != 0) return;

    // 6. 提取通道号并解析
    uint8_t channel = shtp_buf[2]; 
    if (channel == 3 || channel == 4) { // 通道3和4都可能包含传感器数据
        BNO085_ParseSensorReport(&shtp_buf[4], packet_length - 4);
    }
}