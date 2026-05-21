#ifndef __BNO085_HAL_H__
#define __BNO085_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

/* ========================================================================= */
/* 硬件引脚配置 (务必在CubeMX中将SCL和SDA设为 Open-Drain 开漏输出)           */
/* ========================================================================= */
#define I2C_SCL_PORT       GPIOB
#define I2C_SCL_PIN        GPIO_PIN_7
#define I2C_SDA_PORT       GPIOB
#define I2C_SDA_PIN        GPIO_PIN_8

#define BNO085_RST_PORT    GPIOA
#define BNO085_RST_PIN     GPIO_PIN_1
#define BNO085_INT_PORT    GPIOA
#define BNO085_INT_PIN     GPIO_PIN_2

/* 引脚操作宏 */
#define I2C_SCL_H()        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_SET)
#define I2C_SCL_L()        HAL_GPIO_WritePin(I2C_SCL_PORT, I2C_SCL_PIN, GPIO_PIN_RESET)
#define I2C_SDA_H()        HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_SET)
#define I2C_SDA_L()        HAL_GPIO_WritePin(I2C_SDA_PORT, I2C_SDA_PIN, GPIO_PIN_RESET)
#define I2C_SDA_READ()     HAL_GPIO_ReadPin(I2C_SDA_PORT, I2C_SDA_PIN)

/* ========================================================================= */
/* 传感器数据结构体                                                          */
/* ========================================================================= */
typedef struct {
    // 四元数 (归一化后的浮点数)
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
    uint8_t quat_accuracy; // 精度状态: 0=不可靠, 3=最高精度

    // 欧拉角 (单位：度)
    float roll;
    float pitch;
    float yaw;
    
    // 数据更新标志位
    uint8_t data_updated;
} BNO085_Data_t;

// 暴露给外部的数据实例
extern BNO085_Data_t bno085_data;

/* ========================================================================= */
/* 外部调用接口                                                              */
/* ========================================================================= */
uint8_t BNO085_Init(void);
void BNO085_Task(void);

#ifdef __cplusplus
}
#endif

#endif /* __BNO085_H__ */