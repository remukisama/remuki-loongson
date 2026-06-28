#include "car_config.hpp"

static_assert(car_config::kCameraRoiX >= 0 && car_config::kCameraRoiY >= 0,
              "camera ROI origin must be non-negative");
static_assert(car_config::kCameraRoiWidth > 0 && car_config::kCameraRoiHeight > 0,
              "camera ROI dimensions must be positive");
static_assert(car_config::kCameraRoiX + car_config::kCameraRoiWidth <= car_config::kCameraWidth,
              "camera ROI exceeds patrol frame width");
static_assert(car_config::kCameraRoiY + car_config::kCameraRoiHeight <= car_config::kCameraHeight,
              "camera ROI exceeds patrol frame height");
static_assert(car_config::kCameraColumnSampleStep > 0,
              "camera column sample step must be positive");
static_assert(car_config::kCameraRoiHeight <= 60,
              "ROI height exceeds image storage");
static_assert((car_config::kCameraRoiWidth + car_config::kCameraColumnSampleStep - 1) /
                  car_config::kCameraColumnSampleStep <= 80,
              "ROI sampled width exceeds image storage");
