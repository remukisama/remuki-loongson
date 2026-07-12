#include "my_mpu6050.hpp"
#include "car_config.hpp"

#include <chrono>
#include <cmath>
#include <iomanip>
#include <iostream>
#include <unistd.h>

namespace {
constexpr uint64_t kGyroReadIntervalUs = 10000;     // 100 Hz MPU6050 reads; 5 ms control ticks reuse cached data.
constexpr uint64_t kCachedDataMaxAgeUs = 100000;    // 100ms后判定不可用，强制控制环降速/退出
constexpr uint64_t kCachedDataStaleWarnUs = 50000;  // 50ms以上打警告但继续用缓存
constexpr uint64_t kCachedDataSlowdownUs = 50000;   // 50ms以上控制环应降速到弯道速度
constexpr uint32_t kWorkerReadFailLogLimit = 3;

uint64_t steady_now_us() {
    const auto now = std::chrono::steady_clock::now().time_since_epoch();
    return static_cast<uint64_t>(
        std::chrono::duration_cast<std::chrono::microseconds>(now).count());
}
}

MyMpu6050::MyMpu6050()
    : mpu(nullptr),
      gyro_z_bias_raw(0.0f),
      last_temp_c(0.0f),
      temp_read_divider(0),
      last_valid_data{0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f},
      last_read_attempt_us(0),
      last_valid_data_us(0),
      has_last_valid_data(false),
      cached_read_fail_count(0),
      stale_warn_count(0),
      worker_read_fail_count(0),
      async_cache_only(false) {
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
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        temp_read_divider = 0;
        last_read_attempt_us = 0;
        last_valid_data_us = 0;
        has_last_valid_data = false;
        cached_read_fail_count = 0;
        worker_read_fail_count = 0;
    }
    std::cout << "gyro_z_bias_raw = " << gyro_z_bias_raw << std::endl;

    Mpu6050Data initial_data;
    if (!read_raw_sample(initial_data)) {
        std::cerr << "MPU6050 Init Failed! First processed sample invalid." << std::endl;
        return false;
    }
    store_valid_data(initial_data, steady_now_us());
    return true;
}

bool MyMpu6050::read_processed_data(Mpu6050Data& data) {
    if (!mpu) {
        return false;
    }

    uint64_t now_us = steady_now_us();
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        if (async_cache_only) {
            const uint64_t data_age_us =
                (has_last_valid_data && now_us >= last_valid_data_us) ?
                (now_us - last_valid_data_us) : 0;
            if (has_last_valid_data && data_age_us <= kCachedDataMaxAgeUs) {
                data = last_valid_data;
                // stale警告限流：用独立的stale_warn_count，不干扰cached_read_fail_count
                if (data_age_us > kCachedDataStaleWarnUs) {
                    if (stale_warn_count < kWorkerReadFailLogLimit) {
                        ++stale_warn_count;
                        std::cerr << "[MPU6050_GUARD] gyro stale but using cache, age_us="
                                  << data_age_us << std::endl;
                    }
                }
                return true;
            }

            // 超过100ms：返回false，让控制环降速/退出
            if (has_last_valid_data && cached_read_fail_count < kWorkerReadFailLogLimit) {
                ++cached_read_fail_count;
                std::cerr << "[MPU6050_GUARD] async gyro data stale; stop output, age_us="
                          << data_age_us << std::endl;
            }
            return false;
        }

        if (has_last_valid_data &&
            last_read_attempt_us != 0 &&
            now_us >= last_read_attempt_us &&
            now_us - last_read_attempt_us < kGyroReadIntervalUs) {
            data = last_valid_data;
            cached_read_fail_count = 0;
            return true;
        }
    }

    if (!update_async_sample()) {
        now_us = steady_now_us();
        std::lock_guard<std::mutex> lock(data_mutex);
        if (has_last_valid_data &&
            now_us >= last_valid_data_us &&
            now_us - last_valid_data_us <= kCachedDataMaxAgeUs) {
            data = last_valid_data;
            return true;
        }
        return false;
    }

    std::lock_guard<std::mutex> lock(data_mutex);
    if (!has_last_valid_data) {
        return false;
    }
    data = last_valid_data;
    return true;
}

bool MyMpu6050::update_async_sample() {
    if (!mpu) {
        return false;
    }

    const uint64_t attempt_us = steady_now_us();
    {
        std::lock_guard<std::mutex> lock(data_mutex);
        last_read_attempt_us = attempt_us;
    }

    Mpu6050Data sample;
    const bool ok = read_raw_sample(sample);
    if (ok) {
        store_valid_data(sample, steady_now_us());
        worker_read_fail_count = 0;
        return true;
    }

    if (worker_read_fail_count < kWorkerReadFailLogLimit) {
        ++worker_read_fail_count;
        std::cerr << "[MPU6050_WORKER] gyro read failed or invalid; control loop will reuse cache"
                  << std::endl;
    }
    return false;
}

void MyMpu6050::set_async_cache_only(bool enabled) {
    std::lock_guard<std::mutex> lock(data_mutex);
    async_cache_only = enabled;
}

bool MyMpu6050::read_raw_sample(Mpu6050Data& data) {
    if (!mpu) {
        return false;
    }

    int16_t ax, ay, az, gx, gy, gz;
    if (!mpu->get_mpu6050_gyro(&ax, &ay, &az, &gx, &gy, &gz)) {
        return false;
    }

    // Current MPU6050 kernel driver copies a zero-filled buffer if the inner
    // i2c_transfer() timed out but the ioctl still returns success.
    if (ax == 0 && ay == 0 && az == 0 && gx == 0 && gy == 0 && gz == 0) {
        return false;
    }

    data.accel_x = static_cast<float>(ax) / car_config::kMpu6050AccelScale;
    data.accel_y = static_cast<float>(ay) / car_config::kMpu6050AccelScale;
    data.accel_z = static_cast<float>(az) / car_config::kMpu6050AccelScale;
    data.gyro_x = static_cast<float>(gx) / car_config::kMpu6050GyroScale;
    data.gyro_y = static_cast<float>(gy) / car_config::kMpu6050GyroScale;
    data.gyro_z = (static_cast<float>(gz) - gyro_z_bias_raw) / car_config::kMpu6050GyroScale;

    data.temp = last_temp_c;

    if (!std::isfinite(data.gyro_z)) {
        return false;
    }

    return true;
}

void MyMpu6050::store_valid_data(const Mpu6050Data& data, uint64_t timestamp_us) {
    std::lock_guard<std::mutex> lock(data_mutex);
    last_valid_data = data;
    last_valid_data_us = timestamp_us;
    has_last_valid_data = true;
    cached_read_fail_count = 0;
    stale_warn_count = 0;
}
