#include <errno.h>
#include <fcntl.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/ioctl.h>
#include <unistd.h>

#define DEVICE_PATH                 "/dev/lq_i2c_icm42688"
#define I2C_ICM42688_MAGIC          'i'
#define I2C_GET_ICM42688_ID         _IO(I2C_ICM42688_MAGIC, 21)
#define I2C_GET_ICM42688_TEM        _IO(I2C_ICM42688_MAGIC, 22)
#define I2C_GET_ICM42688_GYRO       _IO(I2C_ICM42688_MAGIC, 25)
#define ICM42688_WHO_AM_I           0x47
#define SAMPLE_COUNT                200
#define SAMPLE_INTERVAL_US          5000

static int read_raw_sample(int fd, int16_t data[6])
{
    if (ioctl(fd, I2C_GET_ICM42688_GYRO, data) == 0)
        return 0;

    fprintf(stderr, "[DRIVER_TEST][IOCTL_FAIL] raw errno=%d (%s)\n",
            errno, strerror(errno));
    return -1;
}

static int run_driver_smoke_test(void)
{
    int fd;
    uint8_t id = 0;
    int16_t temperature_raw = 0;
    int16_t data[6] = {0};
    int16_t previous[6] = {0};
    int min_value[6];
    int max_value[6];
    int failures = 0;
    int changed_samples = 0;
    int nonzero_samples = 0;
    int i;
    int axis;

    fd = open(DEVICE_PATH, O_RDWR);
    if (fd < 0) {
        fprintf(stderr, "[DRIVER_TEST][OPEN_FAIL] %s: %s\n",
                DEVICE_PATH, strerror(errno));
        return 2;
    }

    if (ioctl(fd, I2C_GET_ICM42688_ID, &id) != 0) {
        fprintf(stderr, "[DRIVER_TEST][ID_FAIL] errno=%d (%s)\n",
                errno, strerror(errno));
        close(fd);
        return 3;
    }
    printf("[DRIVER_TEST][IDENTITY] WHO_AM_I=0x%02x expected=0x%02x\n",
           id, ICM42688_WHO_AM_I);
    if (id != ICM42688_WHO_AM_I) {
        close(fd);
        return 4;
    }

    if (ioctl(fd, I2C_GET_ICM42688_TEM, &temperature_raw) != 0) {
        fprintf(stderr, "[DRIVER_TEST][TEMP_FAIL] errno=%d (%s)\n",
                errno, strerror(errno));
        close(fd);
        return 5;
    }
    printf("[DRIVER_TEST][TEMPERATURE] raw=%d celsius=%.2f\n",
           temperature_raw, temperature_raw / 132.48 + 25.0);

    if (read_raw_sample(fd, previous) != 0) {
        close(fd);
        return 6;
    }
    for (axis = 0; axis < 6; axis++) {
        min_value[axis] = previous[axis];
        max_value[axis] = previous[axis];
    }

    for (i = 0; i < SAMPLE_COUNT; i++) {
        if (read_raw_sample(fd, data) != 0) {
            failures++;
            usleep(SAMPLE_INTERVAL_US);
            continue;
        }

        if (memcmp(data, previous, sizeof(data)) != 0)
            changed_samples++;
        for (axis = 0; axis < 6; axis++) {
            if (data[axis] != 0)
                nonzero_samples++;
            if (data[axis] < min_value[axis])
                min_value[axis] = data[axis];
            if (data[axis] > max_value[axis])
                max_value[axis] = data[axis];
        }
        memcpy(previous, data, sizeof(previous));
        usleep(SAMPLE_INTERVAL_US);
    }

    printf("[DRIVER_TEST][SAMPLES] count=%d failures=%d changed=%d nonzero_axes=%d\n",
           SAMPLE_COUNT, failures, changed_samples, nonzero_samples);
    printf("[DRIVER_TEST][RANGE] "
           "ax=%d..%d ay=%d..%d az=%d..%d "
           "gx=%d..%d gy=%d..%d gz=%d..%d\n",
           min_value[0], max_value[0], min_value[1], max_value[1],
           min_value[2], max_value[2], min_value[3], max_value[3],
           min_value[4], max_value[4], min_value[5], max_value[5]);

    close(fd);
    if (failures != 0 || changed_samples == 0 || nonzero_samples == 0) {
        fprintf(stderr, "[DRIVER_TEST][FAIL] data path is not healthy\n");
        return 7;
    }

    printf("[DRIVER_TEST][PASS] character-device ioctl path is healthy\n");
    return 0;
}

int main(void)
{
    return run_driver_smoke_test();
}
