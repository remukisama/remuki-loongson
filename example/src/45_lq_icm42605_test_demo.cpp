#include "lq_all_demo.hpp"
#include "motor.hpp"

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

constexpr const char *kI2cDevice = "/dev/i2c-1";
constexpr std::array<uint8_t, 2> kCandidateAddresses{{0x68, 0x69}};

constexpr uint8_t kRegDeviceConfig = 0x11;
constexpr uint8_t kRegTempData1 = 0x1D;
constexpr uint8_t kRegPwrMgmt0 = 0x4E;
constexpr uint8_t kRegGyroConfig0 = 0x4F;
constexpr uint8_t kRegAccelConfig0 = 0x50;
constexpr uint8_t kRegWhoAmI = 0x75;
constexpr uint8_t kRegBankSel = 0x76;

constexpr uint8_t kIcm42605WhoAmI = 0x42;
constexpr uint8_t kIcm42688PWhoAmI = 0x47;
constexpr uint8_t kGyroConfig2000Dps100Hz = 0x08;
constexpr uint8_t kAccelConfig4g100Hz = 0x48;
constexpr uint8_t kGyroConfig2000Dps1kHz = 0x06;
constexpr uint8_t kAccelConfig4g1kHz = 0x46;
constexpr uint8_t kAccelGyroLowNoise = 0x0F;

constexpr double kGyroLsbPerDps = 16.4;
constexpr double kAccelLsbPerG = 8192.0;
constexpr double kTempLsbPerC = 132.48;
constexpr int kSampleCount = 200;
constexpr int kSampleIntervalMs = 10;
constexpr int kPerfBurstReads = 2000;
constexpr int kPerfPacedReads = 3000;
constexpr int kPerfPeriodUs = 1000;

struct RunningStats {
    int count = 0;
    double mean = 0.0;
    double m2 = 0.0;
    double minimum = std::numeric_limits<double>::infinity();
    double maximum = -std::numeric_limits<double>::infinity();

    void add(double value)
    {
        ++count;
        const double delta = value - mean;
        mean += delta / static_cast<double>(count);
        m2 += delta * (value - mean);
        minimum = std::min(minimum, value);
        maximum = std::max(maximum, value);
    }

    double standard_deviation() const
    {
        return count > 1 ? std::sqrt(m2 / static_cast<double>(count - 1)) : 0.0;
    }
};

struct SensorSample {
    int16_t temp_raw = 0;
    int16_t accel_raw[3] = {0, 0, 0};
    int16_t gyro_raw[3] = {0, 0, 0};
    double temp_c = 0.0;
    double accel_g[3] = {0.0, 0.0, 0.0};
    double gyro_dps[3] = {0.0, 0.0, 0.0};
};

bool samples_equal(const SensorSample &left, const SensorSample &right)
{
    if (left.temp_raw != right.temp_raw) {
        return false;
    }
    for (int axis = 0; axis < 3; ++axis) {
        if (left.accel_raw[axis] != right.accel_raw[axis] ||
            left.gyro_raw[axis] != right.gyro_raw[axis]) {
            return false;
        }
    }
    return true;
}

double percentile(std::vector<double> values, double fraction)
{
    if (values.empty()) {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double position = fraction * static_cast<double>(values.size() - 1);
    const std::size_t lower = static_cast<std::size_t>(std::floor(position));
    const std::size_t upper = static_cast<std::size_t>(std::ceil(position));
    const double weight = position - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

int16_t decode_be16(const uint8_t *bytes)
{
    return static_cast<int16_t>((static_cast<uint16_t>(bytes[0]) << 8) |
                                static_cast<uint16_t>(bytes[1]));
}

bool select_i2c_address(int fd, uint8_t address)
{
    if (::ioctl(fd, I2C_SLAVE, address) == 0) {
        return true;
    }
    std::printf("[LQ426XXM][PROBE] address=0x%02X unavailable: %s\n",
                address, std::strerror(errno));
    return false;
}

bool write_register(int fd, uint8_t reg, uint8_t value)
{
    const uint8_t buffer[2] = {reg, value};
    const ssize_t written = ::write(fd, buffer, sizeof(buffer));
    if (written == static_cast<ssize_t>(sizeof(buffer))) {
        return true;
    }
    std::printf("[LQ426XXM][I2C_ERROR] write reg=0x%02X value=0x%02X failed: %s\n",
                reg, value, std::strerror(errno));
    return false;
}

bool read_registers(int fd, uint8_t start_reg, uint8_t *buffer, std::size_t size)
{
    if (::write(fd, &start_reg, 1) != 1) {
        return false;
    }
    return ::read(fd, buffer, size) == static_cast<ssize_t>(size);
}

bool read_register(int fd, uint8_t reg, uint8_t &value)
{
    return read_registers(fd, reg, &value, 1);
}

bool read_sample(int fd, SensorSample &sample)
{
    uint8_t data[14] = {0};
    if (!read_registers(fd, kRegTempData1, data, sizeof(data))) {
        return false;
    }

    sample.temp_raw = decode_be16(data);
    for (int axis = 0; axis < 3; ++axis) {
        sample.accel_raw[axis] = decode_be16(&data[2 + axis * 2]);
        sample.gyro_raw[axis] = decode_be16(&data[8 + axis * 2]);
        sample.accel_g[axis] = static_cast<double>(sample.accel_raw[axis]) / kAccelLsbPerG;
        sample.gyro_dps[axis] = static_cast<double>(sample.gyro_raw[axis]) / kGyroLsbPerDps;
    }
    sample.temp_c = static_cast<double>(sample.temp_raw) / kTempLsbPerC + 25.0;
    return true;
}

bool verify_register(int fd, uint8_t reg, uint8_t expected, const char *name)
{
    uint8_t actual = 0;
    if (!read_register(fd, reg, actual)) {
        std::printf("[LQ426XXM][CONFIG] %s readback failed\n", name);
        return false;
    }
    std::printf("[LQ426XXM][CONFIG] %s=0x%02X expected=0x%02X %s\n",
                name, actual, expected, actual == expected ? "OK" : "MISMATCH");
    return actual == expected;
}

const char *model_name(uint8_t who_am_i)
{
    if (who_am_i == kIcm42605WhoAmI) {
        return "ICM42605";
    }
    if (who_am_i == kIcm42688PWhoAmI) {
        return "ICM42688-P";
    }
    return "unknown";
}

} // namespace

int lq426xxm_test_demo(void)
{
    std::printf("[LQ426XXM][START] bus=%s candidates=0x68,0x69 samples=%d interval_ms=%d\n",
                kI2cDevice, kSampleCount, kSampleIntervalMs);
    std::printf("[LQ426XXM][SAFETY] sensor-only diagnostic; no motor, PWM, encoder, or camera initialization\n");

    const int fd = ::open(kI2cDevice, O_RDWR);
    if (fd < 0) {
        std::printf("[LQ426XXM][FAIL] cannot open %s: %s\n", kI2cDevice, std::strerror(errno));
        return 2;
    }

    uint8_t detected_address = 0;
    uint8_t who_am_i = 0;
    for (const uint8_t address : kCandidateAddresses) {
        if (!select_i2c_address(fd, address)) {
            continue;
        }
        uint8_t value = 0;
        if (read_register(fd, kRegWhoAmI, value)) {
            std::printf("[LQ426XXM][PROBE] address=0x%02X WHO_AM_I=0x%02X\n", address, value);
            if (value == kIcm42605WhoAmI || value == kIcm42688PWhoAmI) {
                detected_address = address;
                who_am_i = value;
                break;
            }
        } else {
            std::printf("[LQ426XXM][PROBE] address=0x%02X no response: %s\n",
                        address, std::strerror(errno));
        }
    }

    if (detected_address == 0) {
        std::printf("[LQ426XXM][FAIL] no supported ICM426xx detected "
                    "(ICM42605=0x%02X, ICM42688-P=0x%02X)\n",
                    kIcm42605WhoAmI, kIcm42688PWhoAmI);
        ::close(fd);
        return 3;
    }

    std::printf("[LQ426XXM][IDENTITY] module=LQ426xxM address=0x%02X "
                "WHO_AM_I=0x%02X chip=%s PASS\n",
                detected_address, who_am_i, model_name(who_am_i));

    bool config_ok = true;
    config_ok = write_register(fd, kRegBankSel, 0x00) && config_ok;
    config_ok = write_register(fd, kRegDeviceConfig, 0x01) && config_ok;
    uint8_t id_after_reset = 0;
    bool reset_ready = false;
    constexpr int kResetRetryCount = 100;
    constexpr int kResetRetryIntervalMs = 10;
    for (int attempt = 1; attempt <= kResetRetryCount; ++attempt) {
        std::this_thread::sleep_for(std::chrono::milliseconds(kResetRetryIntervalMs));
        if (select_i2c_address(fd, detected_address) &&
            read_register(fd, kRegWhoAmI, id_after_reset) &&
            id_after_reset == who_am_i) {
            std::printf("[LQ426XXM][RESET] WHO_AM_I recovered after %d ms\n",
                        attempt * kResetRetryIntervalMs);
            reset_ready = true;
            break;
        }
        if (attempt == 1 || attempt % 10 == 0) {
            std::printf("[LQ426XXM][RESET_WAIT] elapsed_ms=%d WHO_AM_I=0x%02X\n",
                        attempt * kResetRetryIntervalMs, id_after_reset);
        }
    }
    config_ok = reset_ready && config_ok;
    if (!reset_ready) {
        std::printf("[LQ426XXM][FAIL] WHO_AM_I did not recover after soft reset "
                    "within %d ms (last=0x%02X expected=0x%02X)\n",
                    kResetRetryCount * kResetRetryIntervalMs,
                    id_after_reset, who_am_i);
        ::close(fd);
        return 4;
    }

    config_ok = write_register(fd, kRegGyroConfig0, kGyroConfig2000Dps100Hz) && config_ok;
    config_ok = write_register(fd, kRegAccelConfig0, kAccelConfig4g100Hz) && config_ok;
    config_ok = write_register(fd, kRegPwrMgmt0, kAccelGyroLowNoise) && config_ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    config_ok = verify_register(fd, kRegGyroConfig0, kGyroConfig2000Dps100Hz,
                                "GYRO_CONFIG0") && config_ok;
    config_ok = verify_register(fd, kRegAccelConfig0, kAccelConfig4g100Hz,
                                "ACCEL_CONFIG0") && config_ok;
    config_ok = verify_register(fd, kRegPwrMgmt0, kAccelGyroLowNoise,
                                "PWR_MGMT0") && config_ok;
    if (!config_ok) {
        std::printf("[LQ426XXM][FAIL] sensor configuration or readback failed\n");
        ::close(fd);
        return 4;
    }

    std::array<RunningStats, 3> accel_stats;
    std::array<RunningStats, 3> gyro_stats;
    RunningStats temp_stats;
    RunningStats accel_norm_stats;
    int successful_reads = 0;
    int failed_reads = 0;
    int all_zero_reads = 0;
    int repeated_reads = 0;
    bool have_previous = false;
    SensorSample previous;

    for (int sample_index = 0;
         sample_index < kSampleCount && !exit_flag.load(std::memory_order_relaxed);
         ++sample_index) {
        SensorSample sample;
        if (!read_sample(fd, sample)) {
            ++failed_reads;
            if (failed_reads <= 5) {
                std::printf("[LQ426XXM][READ_ERROR] sample=%d error=%s\n",
                            sample_index, std::strerror(errno));
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(kSampleIntervalMs));
            continue;
        }

        ++successful_reads;
        bool all_zero = sample.temp_raw == 0;
        bool same_as_previous = have_previous && sample.temp_raw == previous.temp_raw;
        for (int axis = 0; axis < 3; ++axis) {
            all_zero = all_zero && sample.accel_raw[axis] == 0 && sample.gyro_raw[axis] == 0;
            same_as_previous = same_as_previous &&
                sample.accel_raw[axis] == previous.accel_raw[axis] &&
                sample.gyro_raw[axis] == previous.gyro_raw[axis];
            accel_stats[axis].add(sample.accel_g[axis]);
            gyro_stats[axis].add(sample.gyro_dps[axis]);
        }
        if (all_zero) {
            ++all_zero_reads;
        }
        if (same_as_previous) {
            ++repeated_reads;
        }
        previous = sample;
        have_previous = true;

        const double accel_norm = std::sqrt(
            sample.accel_g[0] * sample.accel_g[0] +
            sample.accel_g[1] * sample.accel_g[1] +
            sample.accel_g[2] * sample.accel_g[2]);
        accel_norm_stats.add(accel_norm);
        temp_stats.add(sample.temp_c);

        if (sample_index % 20 == 0 || sample_index + 1 == kSampleCount) {
            std::printf(
                "[LQ426XXM][SAMPLE] n=%03d temp=%7.2fC accel_g=(%+7.3f,%+7.3f,%+7.3f) |a|=%6.3f gyro_dps=(%+8.3f,%+8.3f,%+8.3f)\n",
                sample_index,
                sample.temp_c,
                sample.accel_g[0], sample.accel_g[1], sample.accel_g[2], accel_norm,
                sample.gyro_dps[0], sample.gyro_dps[1], sample.gyro_dps[2]);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kSampleIntervalMs));
    }

    const bool read_ok = successful_reads >= kSampleCount * 95 / 100;
    const bool data_changes = successful_reads > 1 && repeated_reads < successful_reads - 1;
    const bool no_zero_frames = all_zero_reads == 0;
    const bool temp_plausible = temp_stats.count > 0 &&
                                temp_stats.minimum >= -40.0 && temp_stats.maximum <= 85.0;
    const bool accel_plausible = accel_norm_stats.count > 0 &&
                                 accel_norm_stats.mean >= 0.20 && accel_norm_stats.mean <= 2.00;

    const char *axis_name[3] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        std::printf(
            "[LQ426XXM][STATS] axis=%s accel_mean=%+.5fg accel_std=%.5fg accel_range=[%+.5f,%+.5f] gyro_mean=%+.5fdps gyro_std=%.5fdps gyro_range=[%+.5f,%+.5f]\n",
            axis_name[axis],
            accel_stats[axis].mean, accel_stats[axis].standard_deviation(),
            accel_stats[axis].minimum, accel_stats[axis].maximum,
            gyro_stats[axis].mean, gyro_stats[axis].standard_deviation(),
            gyro_stats[axis].minimum, gyro_stats[axis].maximum);
    }
    std::printf(
        "[LQ426XXM][SUMMARY] model=%s address=0x%02X reads_ok=%d reads_failed=%d "
        "all_zero=%d repeated=%d temp_mean=%.2fC accel_norm_mean=%.4fg\n",
        model_name(who_am_i), detected_address, successful_reads, failed_reads,
        all_zero_reads, repeated_reads, temp_stats.mean, accel_norm_stats.mean);
    std::printf(
        "[LQ426XXM][CHECKS] read_success=%s data_changes=%s no_zero_frames=%s temperature=%s gravity_vector=%s\n",
        read_ok ? "PASS" : "FAIL",
        data_changes ? "PASS" : "FAIL",
        no_zero_frames ? "PASS" : "FAIL",
        temp_plausible ? "PASS" : "FAIL",
        accel_plausible ? "PASS" : "FAIL");

    const bool functional_ok = read_ok && data_changes && no_zero_frames &&
                               temp_plausible && accel_plausible;
    std::printf("[LQ426XXM][FUNCTIONAL_RESULT] %s\n",
                functional_ok ? "PASS" : "FAIL");
    std::printf("[LQ426XXM][IDENTITY_RESULT] PASS (module=LQ426xxM chip=%s)\n",
                model_name(who_am_i));
    std::printf("[LQ426XXM][RESULT] %s\n",
                functional_ok ? "PASS" : "FAIL");

    if (!write_register(fd, kRegPwrMgmt0, 0x00)) {
        std::printf("[LQ426XXM][WARN] failed to return accelerometer and gyroscope to OFF mode\n");
    }
    ::close(fd);
    return functional_ok ? 0 : 5;
}

int lq426xxm_performance_demo(void)
{
    using Clock = std::chrono::steady_clock;

    std::printf("[LQ426XXM][PERF_START] bus=%s odr_hz=1000 burst_reads=%d paced_reads=%d\n",
                kI2cDevice, kPerfBurstReads, kPerfPacedReads);
    std::printf("[LQ426XXM][SAFETY] sensor-only performance diagnostic; "
                "no motor, PWM, encoder, or camera initialization\n");

    const int fd = ::open(kI2cDevice, O_RDWR);
    if (fd < 0) {
        std::printf("[LQ426XXM][PERF_FAIL] cannot open %s: %s\n",
                    kI2cDevice, std::strerror(errno));
        return 2;
    }

    uint8_t detected_address = 0;
    uint8_t who_am_i = 0;
    for (const uint8_t address : kCandidateAddresses) {
        if (!select_i2c_address(fd, address)) {
            continue;
        }
        uint8_t value = 0;
        if (read_register(fd, kRegWhoAmI, value) &&
            (value == kIcm42605WhoAmI || value == kIcm42688PWhoAmI)) {
            detected_address = address;
            who_am_i = value;
            break;
        }
    }
    if (detected_address == 0) {
        std::printf("[LQ426XXM][PERF_FAIL] no supported ICM426xx detected\n");
        ::close(fd);
        return 3;
    }

    std::printf("[LQ426XXM][PERF_IDENTITY] module=LQ426xxM address=0x%02X "
                "WHO_AM_I=0x%02X chip=%s\n",
                detected_address, who_am_i, model_name(who_am_i));

    bool config_ok = true;
    config_ok = write_register(fd, kRegBankSel, 0x00) && config_ok;
    config_ok = write_register(fd, kRegGyroConfig0, kGyroConfig2000Dps1kHz) && config_ok;
    config_ok = write_register(fd, kRegAccelConfig0, kAccelConfig4g1kHz) && config_ok;
    config_ok = write_register(fd, kRegPwrMgmt0, kAccelGyroLowNoise) && config_ok;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    config_ok = verify_register(fd, kRegGyroConfig0, kGyroConfig2000Dps1kHz,
                                "GYRO_CONFIG0_1KHZ") && config_ok;
    config_ok = verify_register(fd, kRegAccelConfig0, kAccelConfig4g1kHz,
                                "ACCEL_CONFIG0_1KHZ") && config_ok;
    config_ok = verify_register(fd, kRegPwrMgmt0, kAccelGyroLowNoise,
                                "PWR_MGMT0") && config_ok;
    if (!config_ok) {
        std::printf("[LQ426XXM][PERF_FAIL] 1 kHz configuration failed\n");
        write_register(fd, kRegPwrMgmt0, 0x00);
        ::close(fd);
        return 4;
    }

    for (int warmup = 0; warmup < 20; ++warmup) {
        SensorSample sample;
        read_sample(fd, sample);
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }

    std::vector<double> burst_latency_us;
    burst_latency_us.reserve(kPerfBurstReads);
    RunningStats burst_latency_stats;
    int burst_success = 0;
    int burst_failed = 0;
    int burst_repeated = 0;
    bool burst_have_previous = false;
    SensorSample burst_previous;
    const auto burst_start = Clock::now();
    for (int index = 0;
         index < kPerfBurstReads && !exit_flag.load(std::memory_order_relaxed);
         ++index) {
        SensorSample sample;
        const auto read_start = Clock::now();
        const bool read_ok = read_sample(fd, sample);
        const auto read_end = Clock::now();
        const double latency_us =
            std::chrono::duration<double, std::micro>(read_end - read_start).count();
        burst_latency_us.push_back(latency_us);
        burst_latency_stats.add(latency_us);
        if (!read_ok) {
            ++burst_failed;
            continue;
        }
        ++burst_success;
        if (burst_have_previous && samples_equal(sample, burst_previous)) {
            ++burst_repeated;
        }
        burst_previous = sample;
        burst_have_previous = true;
    }
    const double burst_elapsed_s =
        std::chrono::duration<double>(Clock::now() - burst_start).count();
    const double burst_rate_hz = burst_elapsed_s > 0.0
        ? static_cast<double>(burst_success) / burst_elapsed_s : 0.0;

    std::vector<double> paced_latency_us;
    std::vector<double> paced_period_us;
    std::vector<double> wake_lateness_us;
    paced_latency_us.reserve(kPerfPacedReads);
    paced_period_us.reserve(kPerfPacedReads);
    wake_lateness_us.reserve(kPerfPacedReads);
    RunningStats paced_latency_stats;
    RunningStats paced_period_stats;
    RunningStats wake_lateness_stats;
    std::array<RunningStats, 3> accel_stats;
    std::array<RunningStats, 3> gyro_stats;
    RunningStats temp_stats;
    RunningStats accel_norm_stats;
    int paced_success = 0;
    int paced_failed = 0;
    int paced_repeated = 0;
    int missed_deadlines = 0;
    bool paced_have_previous = false;
    bool have_last_completion = false;
    SensorSample paced_previous;
    auto last_completion = Clock::now();
    const auto paced_start = Clock::now();
    auto next_deadline = paced_start + std::chrono::microseconds(kPerfPeriodUs);

    for (int index = 0;
         index < kPerfPacedReads && !exit_flag.load(std::memory_order_relaxed);
         ++index) {
        std::this_thread::sleep_until(next_deadline);
        const auto wake_time = Clock::now();
        const double late_us = std::max(
            0.0,
            std::chrono::duration<double, std::micro>(wake_time - next_deadline).count());
        wake_lateness_us.push_back(late_us);
        wake_lateness_stats.add(late_us);
        if (late_us > static_cast<double>(kPerfPeriodUs)) {
            ++missed_deadlines;
        }

        SensorSample sample;
        const auto read_start = Clock::now();
        const bool read_ok = read_sample(fd, sample);
        const auto read_end = Clock::now();
        const double latency_us =
            std::chrono::duration<double, std::micro>(read_end - read_start).count();
        paced_latency_us.push_back(latency_us);
        paced_latency_stats.add(latency_us);
        next_deadline += std::chrono::microseconds(kPerfPeriodUs);

        if (!read_ok) {
            ++paced_failed;
            continue;
        }
        ++paced_success;
        if (have_last_completion) {
            const double period_us =
                std::chrono::duration<double, std::micro>(read_end - last_completion).count();
            paced_period_us.push_back(period_us);
            paced_period_stats.add(period_us);
        }
        last_completion = read_end;
        have_last_completion = true;

        if (paced_have_previous && samples_equal(sample, paced_previous)) {
            ++paced_repeated;
        }
        paced_previous = sample;
        paced_have_previous = true;

        const double accel_norm = std::sqrt(
            sample.accel_g[0] * sample.accel_g[0] +
            sample.accel_g[1] * sample.accel_g[1] +
            sample.accel_g[2] * sample.accel_g[2]);
        accel_norm_stats.add(accel_norm);
        temp_stats.add(sample.temp_c);
        for (int axis = 0; axis < 3; ++axis) {
            accel_stats[axis].add(sample.accel_g[axis]);
            gyro_stats[axis].add(sample.gyro_dps[axis]);
        }
    }

    const double paced_elapsed_s =
        std::chrono::duration<double>(Clock::now() - paced_start).count();
    const int paced_updates = std::max(0, paced_success - paced_repeated);
    const double paced_rate_hz = paced_elapsed_s > 0.0
        ? static_cast<double>(paced_success) / paced_elapsed_s : 0.0;
    const double update_rate_hz = paced_elapsed_s > 0.0
        ? static_cast<double>(paced_updates) / paced_elapsed_s : 0.0;
    const double paced_success_percent = kPerfPacedReads > 0
        ? 100.0 * static_cast<double>(paced_success) / static_cast<double>(kPerfPacedReads)
        : 0.0;

    const double burst_p95_us = percentile(burst_latency_us, 0.95);
    const double burst_p99_us = percentile(burst_latency_us, 0.99);
    const double paced_p95_us = percentile(paced_latency_us, 0.95);
    const double paced_p99_us = percentile(paced_latency_us, 0.99);
    const double period_p95_us = percentile(paced_period_us, 0.95);
    const double period_p99_us = percentile(paced_period_us, 0.99);
    const double wake_p99_us = percentile(wake_lateness_us, 0.99);

    std::printf(
        "[LQ426XXM][PERF_BURST] attempts=%d ok=%d failed=%d repeated=%d "
        "elapsed=%.4fs throughput=%.1f_reads/s latency_us(mean=%.1f p95=%.1f p99=%.1f max=%.1f)\n",
        kPerfBurstReads, burst_success, burst_failed, burst_repeated,
        burst_elapsed_s, burst_rate_hz, burst_latency_stats.mean,
        burst_p95_us, burst_p99_us, burst_latency_stats.maximum);
    std::printf(
        "[LQ426XXM][PERF_PACED] target=1000Hz attempts=%d ok=%d failed=%d "
        "success=%.2f%% repeated=%d updates=%d elapsed=%.4fs "
        "read_rate=%.1fHz update_rate=%.1fHz missed_deadlines=%d\n",
        kPerfPacedReads, paced_success, paced_failed, paced_success_percent,
        paced_repeated, paced_updates, paced_elapsed_s,
        paced_rate_hz, update_rate_hz, missed_deadlines);
    std::printf(
        "[LQ426XXM][PERF_TIMING] read_latency_us(mean=%.1f p95=%.1f p99=%.1f max=%.1f) "
        "period_us(mean=%.1f std=%.1f p95=%.1f p99=%.1f) "
        "wake_late_us(mean=%.1f p99=%.1f max=%.1f)\n",
        paced_latency_stats.mean, paced_p95_us, paced_p99_us,
        paced_latency_stats.maximum,
        paced_period_stats.mean, paced_period_stats.standard_deviation(),
        period_p95_us, period_p99_us,
        wake_lateness_stats.mean, wake_p99_us, wake_lateness_stats.maximum);

    const char *axis_name[3] = {"X", "Y", "Z"};
    for (int axis = 0; axis < 3; ++axis) {
        std::printf(
            "[LQ426XXM][PERF_NOISE] axis=%s accel_mean=%+.5fg accel_std=%.3fmg "
            "gyro_mean=%+.5fdps gyro_std=%.5fdps\n",
            axis_name[axis], accel_stats[axis].mean,
            accel_stats[axis].standard_deviation() * 1000.0,
            gyro_stats[axis].mean, gyro_stats[axis].standard_deviation());
    }
    std::printf(
        "[LQ426XXM][PERF_ENV] temp_mean=%.2fC accel_norm_mean=%.5fg\n",
        temp_stats.mean, accel_norm_stats.mean);

    const bool zero_i2c_errors = burst_failed == 0 && paced_failed == 0;
    const bool success_rate_ok = paced_success_percent >= 99.0;
    const bool read_rate_ok = paced_rate_hz >= 900.0;
    const bool update_rate_ok = update_rate_hz >= 800.0;
    const bool latency_ok = paced_p99_us < 1000.0;
    const bool gravity_ok = accel_norm_stats.count > 0 &&
                            accel_norm_stats.mean >= 0.20 &&
                            accel_norm_stats.mean <= 2.00;
    const bool overall_ok = zero_i2c_errors && success_rate_ok && read_rate_ok &&
                            update_rate_ok && latency_ok && gravity_ok;
    std::printf(
        "[LQ426XXM][PERF_CHECKS] zero_i2c_errors=%s success_rate=%s "
        "read_rate_900hz=%s update_rate_800hz=%s latency_p99_lt_1ms=%s gravity=%s\n",
        zero_i2c_errors ? "PASS" : "FAIL",
        success_rate_ok ? "PASS" : "FAIL",
        read_rate_ok ? "PASS" : "FAIL",
        update_rate_ok ? "PASS" : "FAIL",
        latency_ok ? "PASS" : "FAIL",
        gravity_ok ? "PASS" : "FAIL");
    std::printf("[LQ426XXM][PERF_RESULT] %s\n", overall_ok ? "PASS" : "FAIL");

    if (!write_register(fd, kRegPwrMgmt0, 0x00)) {
        std::printf("[LQ426XXM][WARN] failed to return sensor to OFF mode\n");
    }
    ::close(fd);
    return overall_ok ? 0 : 6;
}
