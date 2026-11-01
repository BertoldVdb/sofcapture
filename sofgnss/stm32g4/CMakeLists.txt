# STM32G474 firmware (bare metal, TinyUSB).
#   cmake -S stm32g4 -B stm32g4/build && cmake --build stm32g4/build
#   cmake --build stm32g4/build --target dfu       # bootloader and app over USB, board in DFU mode
#   cmake --build stm32g4/build --target flash     # the same over ST-LINK
# sofgnss-signed.bin is the OTA image. The signing key is a P-256 PEM
# (sign.py keygen), set with -DSOFGNSS_STM32_KEY=.
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
FetchContent_Declare(micro_ecc
    URL https://github.com/kmackay/micro-ecc/archive/refs/tags/v1.1.tar.gz
    URL_HASH SHA256=0e39fba553f6ae1870c3033201d3eaa61a59bf4aa3bdbabed7a26e1bb5b7e890)
# Sources only: none of them is built through its own CMakeLists.
foreach(dep tinyusb cmsis_device_g4 cmsis micro_ecc)
  FetchContent_GetProperties(${dep})
  if(NOT ${dep}_POPULATED)
    FetchContent_Populate(${dep})
  endif()
endforeach()

set(SOFGNSS_STM32_KEY ${CMAKE_CURRENT_SOURCE_DIR}/../keys/stm32_signing_key.pem CACHE FILEPATH
    "P-256 key that signs the app")
set(CORE ${CMAKE_CURRENT_SOURCE_DIR}/../core)
set(TUSB ${tinyusb_SOURCE_DIR}/src)

# At every build, not just at configure time.
set(VERSION_H ${CMAKE_CURRENT_BINARY_DIR}/sofgnss_version.h)
add_custom_target(sofgnss_version
    COMMAND ${CMAKE_COMMAND} -DSRC=${CMAKE_CURRENT_SOURCE_DIR} -DOUT=${VERSION_H}
            -P ${CMAKE_CURRENT_SOURCE_DIR}/version.cmake
    BYPRODUCTS ${VERSION_H})

set(USB_SOURCES
    ${TUSB}/tusb.c
    ${TUSB}/common/tusb_fifo.c
    ${TUSB}/device/usbd.c
    ${TUSB}/device/usbd_control.c
    ${TUSB}/class/cdc/cdc_device.c
    ${TUSB}/portable/st/stm32_fsdev/dcd_stm32_fsdev.c
    ${cmsis_device_g4_SOURCE_DIR}/Source/Templates/gcc/startup_stm32g474xx.s)

set(CPU -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard)

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
    ${CORE}/src/cal_signal.cpp
    ${USB_SOURCES}
)
add_dependencies(sofgnss.elf sofgnss_version)
target_include_directories(sofgnss.elf PRIVATE include ${CMAKE_CURRENT_BINARY_DIR})
target_link_options(sofgnss.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/stm32g474xb.ld -Wl,-Map=sofgnss.map)
set_target_properties(sofgnss.elf PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/stm32g474xb.ld)

# The .bin carries the ELF's SHA-256 in its app descriptor, as on the ESP32-S3.
add_custom_command(TARGET sofgnss.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary sofgnss.elf sofgnss.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/app_desc.py sofgnss.elf sofgnss.bin
    COMMAND ${CMAKE_SIZE} sofgnss.elf
    BYPRODUCTS sofgnss.bin)

# ---- Bootloader ---------------------------------------------------------------

add_custom_command(OUTPUT boot_key.h
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/sign.py pubkey ${SOFGNSS_STM32_KEY} boot_key.h
    DEPENDS ${SOFGNSS_STM32_KEY} ${CMAKE_CURRENT_SOURCE_DIR}/sign.py)

add_executable(boot.elf
    boot/main.cpp
    boot/sha256.c
    src/usb_descriptors.c
    ${CORE}/src/pvdb.cpp
    ${CORE}/src/ota_receiver.cpp
    ${micro_ecc_SOURCE_DIR}/uECC.c
    ${USB_SOURCES}
    ${CMAKE_CURRENT_BINARY_DIR}/boot_key.h
)
target_include_directories(boot.elf PRIVATE
    boot
    include
    ${CMAKE_CURRENT_BINARY_DIR}
    ${micro_ecc_SOURCE_DIR})
set_source_files_properties(${micro_ecc_SOURCE_DIR}/uECC.c PROPERTIES COMPILE_DEFINITIONS
    "uECC_SUPPORTS_secp160r1=0;uECC_SUPPORTS_secp192r1=0;uECC_SUPPORTS_secp224r1=0;uECC_SUPPORTS_secp256k1=0;uECC_SUPPORT_COMPRESSED_POINT=0;uECC_OPTIMIZATION_LEVEL=2")
target_link_options(boot.elf PRIVATE
    -T${CMAKE_CURRENT_SOURCE_DIR}/boot/boot.ld -Wl,-Map=boot.map)
set_target_properties(boot.elf PROPERTIES LINK_DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/boot/boot.ld)
add_custom_command(TARGET boot.elf POST_BUILD
    COMMAND ${CMAKE_OBJCOPY} -O binary boot.elf boot.bin
    COMMAND ${CMAKE_SIZE} boot.elf
    BYPRODUCTS boot.bin)

foreach(t sofgnss.elf boot.elf)
  target_include_directories(${t} PRIVATE
      ${CORE}/include
      ${TUSB}
      ${cmsis_device_g4_SOURCE_DIR}/Include
      ${cmsis_SOURCE_DIR}/CMSIS/Core/Include)
  target_compile_definitions(${t} PRIVATE STM32G474xx)
  target_compile_options(${t} PRIVATE
      ${CPU} -ffunction-sections -fdata-sections -Wall -g
      $<$<COMPILE_LANGUAGE:CXX>:-std=gnu++17 -fno-exceptions -fno-rtti -fno-threadsafe-statics
                                -fno-use-cxa-atexit>)
  target_link_options(${t} PRIVATE ${CPU} -Wl,--gc-sections --specs=nano.specs --specs=nosys.specs)
endforeach()

# The OTA image, and everything for an empty chip.
add_custom_command(OUTPUT sofgnss-signed.bin sofgnss-full.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/sign.py sign ${SOFGNSS_STM32_KEY} sofgnss.bin sofgnss-signed.bin
    COMMAND python3 ${CMAKE_CURRENT_SOURCE_DIR}/sign.py full boot.bin sofgnss-signed.bin sofgnss-full.bin
    DEPENDS sofgnss.elf boot.elf ${SOFGNSS_STM32_KEY} ${CMAKE_CURRENT_SOURCE_DIR}/sign.py)
add_custom_target(images ALL DEPENDS sofgnss-signed.bin sofgnss-full.bin)

# dfu-util fails the status request after :leave because the chip has already
# started the bootloader, so a completed download counts as success.
add_custom_target(dfu
    COMMAND sh -c "dfu-util -a 0 -s 0x08000000:leave -D sofgnss-full.bin 2>&1 | tee /dev/stderr | grep -q 'File downloaded successfully'"
    DEPENDS images
    USES_TERMINAL)

add_custom_target(flash
    COMMAND openocd -f interface/stlink.cfg -f target/stm32g4x.cfg
            -c "program sofgnss-full.bin 0x08000000 verify reset exit"
    DEPENDS images
    USES_TERMINAL)
