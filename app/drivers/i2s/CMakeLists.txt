# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/i2s.h)

zephyr_library()

zephyr_library_sources(sam0_i2s.c)
