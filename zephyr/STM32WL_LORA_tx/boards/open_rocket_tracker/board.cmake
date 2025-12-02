# SPDX-License-Identifier: Apache-2.0

# OpenOCD configuration for ST-LINK V3
board_runner_args(openocd "--target-handle=_CHIPNAME.cpu")

board_runner_args(pyocd "--target=stm32wle5jcix")
board_runner_args(pyocd "--flash-opt=-O reset_type=hw")
board_runner_args(pyocd "--flash-opt=-O connect_mode=under-reset")
board_runner_args(jlink "--device=STM32WLE5JC" "--speed=4000" "--reset-after-load")
board_runner_args(stm32cubeprogrammer "--port=swd" "--reset-mode=hw")
board_runner_args(blackmagicprobe "--connect-rst")

# OpenOCD first for ST-LINK V3 default
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/stm32cubeprogrammer.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/blackmagicprobe.board.cmake)
