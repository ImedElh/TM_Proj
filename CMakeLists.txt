# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(FirstBleProject)

target_sources(app PRIVATE src/main.c)

# Custom files and folders

target_sources(app PRIVATE
    src/remote_service/remote.c
    src/vl53l1/vl53l1.c
	src/adxl345/accel0.c
)

zephyr_library_include_directories(src/remote_service)
zephyr_library_include_directories(src/vl53l1)
zephyr_library_include_directories(src/adxl345)
