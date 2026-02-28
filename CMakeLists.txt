# The following lines of boilerplate have to be in your project's CMakeLists
# in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)

list(APPEND EXTRA_COMPONENT_DIRS "odrive_ascii")
list(APPEND EXTRA_COMPONENT_DIRS "as5048a")


include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(uart_async_rxtxtasks)


set(TARGET_ELF "${CMAKE_PROJECT_NAME}.elf")


set(ESPTOOL "$ENV{IDF_PATH}/components/esptool_py/esptool/esptool.py")
set(BOOTLOADER_BIN "${BUILD_DIR}/bootloader/bootloader.bin")
set(PARTITION_TABLE_BIN "${BUILD_DIR}/partition_table/partition-table.bin")
set(PROJECT_BIN "${BUILD_DIR}/${CMAKE_PROJECT_NAME}.bin")
set(TARGET_QEMU_BINARY "${BUILD_DIR}/firmware.bin")


add_custom_target(firmware_bin ALL 
DEPENDS ${TARGET_QEMU_BINARY}  )

add_custom_command(
    OUTPUT ${TARGET_QEMU_BINARY}
    COMMAND python ${ESPTOOL} --chip esp32 merge_bin --output ${TARGET_QEMU_BINARY} --fill-flash-size 8MB  0x10000 ${PROJECT_BIN} 0x1000 ${BOOTLOADER_BIN} 0x8000 ${PARTITION_TABLE_BIN} --flash_mode dio --flash_freq 40m --flash_size 8MB
    DEPENDS app
    COMMENT "Generating binary for QEMU"
)

