# SPDX-License-Identifier: Apache-2.0

board_runner_args(jlink "--device=MK66FN2M0xxx18")
board_runner_args(linkserver  "--device=MK66FN2M0xxx18:FRDM-K66F")

board_runner_args(pyocd "--target=mk66fn2m0vmd18")

include(${ZEPHYR_BASE}/boards/common/linkserver.board.cmake)
include(${ZEPHYR_BASE}/boards/common/pyocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/jlink.board.cmake)
include(${ZEPHYR_BASE}/boards/common/openocd.board.cmake)
include(${ZEPHYR_BASE}/boards/common/canopen.board.cmake)
