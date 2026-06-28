#ifndef __MY_MPU6050_HPP
#define __MY_MPU6050_HPP

#include "lq_i2c_mpu6050.hpp"
#include <cstdint>

struct Mpu6050Data {
    float accel_x;
    float accel_y;
    float accel_z;
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float temp;
};

class MyMpu6050 {
public:
    MyMpu6050();
    ~MyMpu6050();

    bool init();
    bool read_processed_data(Mpu6050Data& data);

private:
    lq_i2c_mpu6050* mpu;
    float gyro_z_bias_raw;
    float last_temp_c;
    uint16_t temp_read_divider;
};

#endif
