# SPDX-License-Identifier: Apache-2.0
cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(wireless_ecu_console)

target_sources(app PRIVATE
    src/main.c
    src/vehicle_protocol.c
    src/usb_cdc_transport.c
    src/ble_nus_transport.c
)

target_include_directories(app PRIVATE src)
