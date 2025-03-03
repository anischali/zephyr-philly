# SPDX-License-Identifier: Apache-2.0

zephyr_syscall_header(${ZEPHYR_BASE}/include/zephyr/drivers/i2s.h)

zephyr_library_amend()

zephyr_library_sources_ifdef(CONFIG_I2S_SAM0 i2s_sam0.c)