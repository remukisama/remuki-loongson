#ifndef __MY_MPU6050_HPP
#define __MY_MPU6050_HPP

#include "lq_i2c_mpu6050.hpp"

// 结构体保存处理后的物理量数据
struct Mpu6050Data {
    float accel_x; // 加速度 X轴 (g)
    float accel_y; // 加速度 Y轴 (g)
    float accel_z; // 加速度 Z轴 (g)
    float gyro_x;  // 角速度 X轴 (°/s)
    float gyro_y;  // 角速度 Y轴 (°/s)
    float gyro_z;  // 角速度 Z轴 (°/s)
    float temp;    // 温度 (℃)
};

class MyMpu6050 {
public:
    MyMpu6050();
    ~MyMpu6050();

    bool init();
    bool read_processed_data(Mpu6050Data& data);

private:
    lq_i2c_mpu6050* mpu;
    float gyro_z_bias_raw; // 静止零偏(原始LSB)
};

#endif
