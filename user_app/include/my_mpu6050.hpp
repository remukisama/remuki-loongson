#ifndef __MY_MPU6050_HPP
#define __MY_MPU6050_HPP

#include "lq_i2c_mpu6050.hpp"
#include <cstdint>
#include <mutex>

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
    bool update_async_sample();
    void set_async_cache_only(bool enabled);

private:
    bool read_raw_sample(Mpu6050Data& data);
    void store_valid_data(const Mpu6050Data& data, uint64_t timestamp_us);

    lq_i2c_mpu6050* mpu;
    float gyro_z_bias_raw;
    float last_temp_c;
    uint16_t temp_read_divider;
    Mpu6050Data last_valid_data;
    uint64_t last_read_attempt_us;
    uint64_t last_valid_data_us;
    bool has_last_valid_data;
    uint32_t cached_read_fail_count;   // 超过MaxAge的硬失败计数（控制环据此决定停车）
    uint32_t stale_warn_count;         // 50-100ms stale窗口内的警告限流计数（独立递增）
    uint32_t worker_read_fail_count;
    bool async_cache_only;
    std::mutex data_mutex;
};

#endif
