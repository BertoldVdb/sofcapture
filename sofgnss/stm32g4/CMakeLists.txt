# STM32G474 firmware (bare metal, TinyUSB).
#   cmake -S stm32g4 -B stm32g4/build && cmake --build stm32g4/build
#   cmake --build stm32g4/build --target flash     # over ST-LINK
cmake_minimum_required(VERSION 3.16)
set(CMAKE_TOOLCHAIN_FILE ${CMAKE_CURRENT_SOURCE_DIR}/arm-none-eabi.cmake)
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE MinSizeRel)
endif()
project(sofgnss C CXX ASM)

include(FetchContent)
FetchContent_Declare(tinyusb
    URL https://github.com/hathach/tinyusb/archive/refs/tags/0.19.0.tar.gz
    URL_HASH SHA256=d66b3df59d3d7cd511cda890cc633940e896e2ddf9e1cca6731aa341d3a07bd2)
FetchContent_Declare(cmsis_device_g4
    URL https://github.com/STMicroelectronics/cmsis-device-g4/archive/refs/tags/v1.2.6.tar.gz
    URL_HASH SHA256=a150d4b334610d8d8445385e9104d08aa7ae8c4e443b2a7dc6f19c9711d8bb4c)
# CMSIS 6 needs a newer GCC than the 8.3 in Debian.
FetchContent_Declare(cmsis
    URL https://github.com/ARM-software/CMSIS_5/archive/refs/tags/5.9.0.tar.gz
    URL_HASH SHA256=9ce693dd15c41f02506adda79323c077912b3211303ca0d9b28b8ebe735a7031)
# Sources only: none of them is built through its own CMakeLists.
foreach(dep tinyusb cmsis_device_g4 cmsis)
  FetchContent_GetProperties(${dep})
  if(NOT ${dep}_POPULATED)
    FetchContent_Populate(${dep})
  endif()
endforeach()

set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../core)
set(TUSB ${tinyusb_SOURCE_DIR}/src)

execute_process(COMMAND git describe --always --tags --dirty
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    OUTPUT_VARIABLE SOFGNSS_VERSION OUTPUT_STRIP_TRAILING_WHITESPACE)

add_executable(sofgnss.elf
    src/cal.cpp
    src/main.cpp
    src/system.cpp
    src/timer.cpp
    src/uart.cpp
    src/usb.cpp
    src/usb_descriptors.c
    ${CORE}/src/frame_parser.cpp
    ${CORE}/src/msg_queue.cpp
    ${CORE}/src/pvdb.cpp
    ${CORE}/src/timing_reporter.cpp
    ${CORE}/src/ota_receiver.cpp
    ${CORE}/src/cal_signal.cpp
    ${TUSB}/tusb.c
    ${TUSB}/common/tusb_fifo.c
    ${TUSB}/device/usbd.c
    ${TUSB}/device/usbd_control.c
    ${TUSB}/class/cdc/cdc_device.c
    ${TUSB}/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ${cmsis_device_g4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s
)
target_include_directories(sofgnss.elf PRIVATE
    include
    ${CORE}/include
    ${TUSB}
    ${cmsis_device_g4_SOURCE_DIR}/Include
    ${cmsis_SOURCE_DIR}/CMSIS/Core/Include)
target_compile_definitions(sofgnss.elf PRIVATE
    STM32G474xx
    SOFGNSS_VERSION="${SOFGNSS_VERSION}")

set(CPU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)
target_compile_options(sofgnss.elf PRIVATE
    ${CPU} -ffunction-sections -fdata-sections -Wall -g
    $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics
                              -fno-use-cxa-atexit>)
target_link_options(sofgnss.elf PRIVATE
    ${CPU} -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g474xb.ld -Wl,--gc-sections
    -Wl,-Map=sofgnss.map --specs=nano.specs --specs=nosys.specs)
set_target_properties(sofgnss.elf PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32g474xb.ld)

# The .bin carries the ELF's SHA-256 in its app descriptor, as on the ESP32-S3.
add_custom_command(TARGET sofgnss.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary sofgnss.elf sofgnss.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/app_desc.py sofgnss.elf sofgnss.bin
    COMMAND ${CMAKE_SIZE} sofgnss.elf
    BYPRODUCTS sofgnss.bin)

add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32g4x.cfg
            -c "program sofgnss.bin 0x08000000 verify reset exit"
    DEPENDS sofgnss.elf
    USES_TERMINAL)
