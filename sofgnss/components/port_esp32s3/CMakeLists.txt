idf_component_register(
    SRCS
        src/usb_port.cpp
        src/usb_descriptors.c
        src/timer_port.cpp
        src/uart_port.cpp
        src/sys_port.cpp
        src/ota_port.cpp
        src/cal_port.cpp
    INCLUDE_DIRS include
    REQUIRES core
    PRIV_REQUIRES esp_driver_mcpwm esp_driver_uart esp_driver_gpio esp_timer usb app_update esp_app_format mbedtls esp_driver_spi esp_driver_ledc
)

# TinyUSB finds tusb_config.h here.
idf_component_get_property(tusb_lib espressif__tinyusb COMPONENT_LIB)
target_include_directories(${tusb_lib} PRIVATE "${CMAKE_CURRENT_LIST_DIR}/include")

# Only TinyUSB calls these, so force the linker to keep them.
target_link_libraries(${COMPONENT_LIB} INTERFACE
    "-u tud_descriptor_device_cb -u tud_descriptor_configuration_cb -u tud_descriptor_string_cb")
