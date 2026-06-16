#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "${ROOT_DIR}"

TOOLCHAIN="${ROOT_DIR}/tools/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6"
CXX="${TOOLCHAIN}/bin/loongarch64-linux-gnu-g++"
OUT="${ROOT_DIR}/board_tools/ncnn_capture_infer_main"

export PKG_CONFIG_PATH="${ROOT_DIR}/tools/LQ_Dep_libs/opencv_install/lib/pkgconfig:${ROOT_DIR}/tools/LQ_Dep_libs/ncnn_install/lib/pkgconfig"

read -r -a CV_CFLAGS <<< "$(pkg-config --cflags opencv4)"
read -r -a CV_LIBS <<< "$(pkg-config --libs opencv4)"
read -r -a NCNN_CFLAGS <<< "$(pkg-config --cflags ncnn)"
read -r -a NCNN_LIBS <<< "$(pkg-config --libs ncnn)"

"${CXX}" -std=c++17 -O2 -DLQ_HAVE_OPENCV \
  -Imain \
  -Iuser_app/include \
  -Ilibraries/app \
  -Ilibraries/app/gyro \
  -Ilibraries/app/display \
  -Ilibraries/common/inc \
  -Ilibraries/drv/inc \
  "${CV_CFLAGS[@]}" \
  "${NCNN_CFLAGS[@]}" \
  board_tools/ncnn_capture_infer_main.cpp \
  user_app/src/red_marker_detect.cpp \
  -Llibraries/drv/src \
  -llqncnn \
  -llq_camera_ex \
  "${CV_LIBS[@]}" \
  "${NCNN_LIBS[@]}" \
  -lpthread \
  -lrt \
  -o "${OUT}"

file "${OUT}"
ls -l "${OUT}"
