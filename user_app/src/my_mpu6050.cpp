#include "my_mpu6050.hpp"
#include "car_config.hpp"
#include <iostream>
#include <iomanip>
#include <unistd.h>

MyMpu6050::MyMpu6050() {
    mpu = nullptr;  // 延迟到 init() 再创建，避免全局构造时打开不存在的 I2C 设备导致崩溃
    gyro_z_bias_raw = 0.0f;
}

MyMpu6050::~MyMpu6050() {
    if (mpu) {
        delete mpu;
        mpu = nullptr;
    }
}

bool MyMpu6050::init() {
    if (!mpu) {
        mpu = new lq_i2c_mpu6050(MPU6050_DEV_NAME);
    }
    uint8_t id = mpu->get_mpu6050_id();
    // MPU6050 默认 Device ID 是 0x68
    if (id == 0x68) {
        std::cout << "MPU6050 Init Success! Device ID: 0x" << std::hex << (int)id << std::dec << std::endl;
        double bias_sum = 0.0;
        std::cout << "开始采样陀螺仪零偏，请保持静止..." << std::endl;
        for (int i = 0; i < car_config::kMpu6050GyroBiasSamples; ++i) {
            int16_t ax, ay, az, gx, gy, gz;
            if (!mpu->get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
                return false;
            }
            bias_sum += gz;
            usleep(car_config::kMpu6050BiasSampleDelayUs);
        }
        gyro_z_bias_raw = static_cast<float>(bias_sum / car_config::kMpu6050GyroBiasSamples);
        std::cout << "gyro_z_bias_raw = " << gyro_z_bias_raw << std::endl;
        return true;
    } else {
        std::cerr << "MPU6050 Init Failed! Device ID (read): 0x" << std::hex << (int)id << std::dec << " (expected 0x68)" << std::endl;
        return false;
    }
}

bool MyMpu6050::read_processed_data(Mpu6050Data& data) {
    if (!mpu) return false;

    int16_t ax, ay, az, gx, gy, gz;

    // 1. 读取 MPU6050 原始 int16_t 数据
    if (!mpu->get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        return false;
    }

    // 2. 将原始加速度数据(ADC值)转换为物理单位 (g, 1g=9.8m/s²)
    data.accel_x = (float)ax / car_config::kMpu6050AccelScale;
    data.accel_y = (float)ay / car_config::kMpu6050AccelScale;
    data.accel_z = (float)az / car_config::kMpu6050AccelScale;

    // 3. 将原始角速度数据(ADC值)转换为物理单位 (°/s, 度/秒)
    data.gyro_x = (float)gx / car_config::kMpu6050GyroScale;
    data.gyro_y = (float)gy / car_config::kMpu6050GyroScale;
    data.gyro_z = ((float)gz - gyro_z_bias_raw) / car_config::kMpu6050GyroScale;

    // 4. 读取处理好的温度数据 (自带除以100并转float的处理)
    data.temp = mpu->get_mpu6050_tem();

    return true;
}
