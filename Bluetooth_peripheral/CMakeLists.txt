#
# Copyright (c) 2018 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#
cmake_minimum_required(VERSION 3.20.0)
set(BOARD_ROOT ${CMAKE_SOURCE_DIR})
set(DTC_OVERLAY_FILE ${CMAKE_SOURCE_DIR}/boards/nrf5340dk_nrf5340_cpuapp_ns.overlay)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(NONE)

# NORDIC SDK APP START
target_sources(app PRIVATE
  src/main.c
  src/bluetooth.c
)

# NORDIC SDK APP END
zephyr_library_include_directories(.)
