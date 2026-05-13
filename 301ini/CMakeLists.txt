#
# Copyright (c) 2024 Nordic Semiconductor
#
# SPDX-License-Identifier: LicenseRef-Nordic-5-Clause
#

cmake_minimum_required(VERSION 3.20.0)

find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(301ini)
# 自动查找 src/ 根目录下的所有 .c 文件
FILE(GLOB app_sources src/*.c)
# 手动添加 src/flash/ 子目录中的文件
FILE(GLOB flash_sources src/flash/*.c)
FILE(GLOB button_led_sources src/interface/*.c)

# NORDIC SDK APP START
target_sources(app PRIVATE
	${app_sources}
	${flash_sources}
	${button_led_sources}
)
# NORDIC SDK APP END
