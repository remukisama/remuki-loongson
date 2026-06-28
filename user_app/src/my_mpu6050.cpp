#include "my_mpu6050.hpp"
#include "car_config.hpp"

#include <cmath>
#include <iomanip>
#include <iostream>
#include <unistd.h>

namespace {
constexpr uint16_t kTemperatureReadDivider = 200;  // 1 Hz at a 5 ms control period
}

MyMpu6050::MyMpu6050()
    : mpu(nullptr),
      gyro_z_bias_raw(0.0f),
      last_temp_c(0.0f),
      temp_read_divider(0) {
}

MyMpu6050::~MyMpu6050() {
    delete mpu;
    mpu = nullptr;
}

bool MyMpu6050::init() {
    if (!mpu) {
        mpu = new lq_i2c_mpu6050(MPU6050_DEV_NAME);
    }

    const uint8_t id = mpu->get_mpu6050_id();
    if (id != 0x68) {
        std::cerr << "MPU6050 Init Failed! Device ID (read): 0x"
                  << std::hex << static_cast<int>(id) << std::dec
                  << " (expected 0x68)" << std::endl;
        return false;
    }

    std::cout << "MPU6050 Init Success! Device ID: 0x"
              << std::hex << static_cast<int>(id) << std::dec << std::endl;
    std::cout << "开始采样陀螺仪零偏，请保持静止..." << std::endl;

    double bias_sum = 0.0;
    for (int i = 0; i < car_config::kMpu6050GyroBiasSamples; ++i) {
        int16_t ax, ay, az, gx, gy, gz;
        if (!mpu->get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
            return false;
        }
        bias_sum += gz;
        usleep(car_config::kMpu6050BiasSampleDelayUs);
    }

    gyro_z_bias_raw = static_cast<float>(bias_sum / car_config::kMpu6050GyroBiasSamples);
    temp_read_divider = 0;
    std::cout << "gyro_z_bias_raw = " << gyro_z_bias_raw << std::endl;
    return true;
}

bool MyMpu6050::read_processed_data(Mpu6050Data& data) {
    if (!mpu) {
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;
    if (!mpu->get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        return false;
    }

    data.accel_x = static_cast<float>(ax) / car_config::kMpu6050AccelScale;
    data.accel_y = static_cast<float>(ay) / car_config::kMpu6050AccelScale;
    data.accel_z = static_cast<float>(az) / car_config::kMpu6050AccelScale;
    data.gyro_x = static_cast<float>(gx) / car_config::kMpu6050GyroScale;
    data.gyro_y = static_cast<float>(gy) / car_config::kMpu6050GyroScale;
    data.gyro_z = (static_cast<float>(gz) - gyro_z_bias_raw) / car_config::kMpu6050GyroScale;

    // Temperature is diagnostic-only. Do not add a second I2C transaction to each
    // 5 ms control iteration; publish a cached value instead.
    if (temp_read_divider == 0) {
        const float temperature = mpu->get_mpu6050_tem();
        if (std::isfinite(temperature)) {
            last_temp_c = temperature;
        }
        temp_read_divider = kTemperatureReadDivider - 1;
    } else {
        --temp_read_divider;
    }
    data.temp = last_temp_c;

    return std::isfinite(data.gyro_z);
}
