# SPDX-License-Identifier: Apache-2.0

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(i2c_custom_target)

target_sources(app PRIVATE src/main.c)

add_subdirectory(app_components/i2c_app)
add_subdirectory(app_components/audio_i2s_app)
#add_subdirectory(app_components/utils)

target_include_directories(app PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/app_components/audio_i2s_app
    ${CMAKE_CURRENT_SOURCE_DIR}/app_components/i2c_app
)

target_link_libraries(app PRIVATE
    audio_i2s_app
    i2c_app
)
