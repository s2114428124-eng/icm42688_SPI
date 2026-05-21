#ifndef __ICM42688P_HAL_H__
#define __ICM42688P_HAL_H__

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "stdio.h"
/* ========================================================================= */
/* 硬件引脚配置 (务必在CubeMX中将SCL和SDA设为 Open-Drain 开漏输出)           */
/* ========================================================================= */
#define I2C_SCL_PORT       GPIOC
#define I2C_SCL_PIN        GPIO_PIN_0
#define I2C_SDA_PORT       GPIOB
#define I2C_SDA_PIN        GPIO_PIN_7

// 根据实际情况保留复位和中断引脚
#define ICM_INT_PORT       GPIOA
#define ICM_INT_PIN        GPIO_PIN_2

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
    // 原始数据
    int16_t accel_raw_x;
    int16_t accel_raw_y;
    int16_t accel_raw_z;
    int16_t gyro_raw_x;
    int16_t gyro_raw_y;
    int16_t gyro_raw_z;
    int16_t temp_raw;

    // 物理量转换后的数据
    float accel_x;  // 单位: g
    float accel_y;
    float accel_z;
    float gyro_x;   // 单位: dps (度/秒)
    float gyro_y;
    float gyro_z;
    float temp;     // 单位: ℃

    uint8_t data_updated;
} ICM42688P_Data_t;

// 暴露给外部的数据实例
extern ICM42688P_Data_t icm_data;

/* ========================================================================= */
/* 外部调用接口                                                              */
/* ========================================================================= */
uint8_t ICM42688P_Init(void);
void ICM42688P_Task(void);
void ICM42688P_Offset(void);
#ifdef __cplusplus
}
#endif

#endif /* __ICM42688P_HAL_H__ */