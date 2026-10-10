cmake_minimum_required(VERSION 3.16)

# The portable core, shared with the STM32 port.
set(EXTRA_COMPONENT_DIRS ${CMAKE_CURRENT_LIST_DIR}/../core)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(sofgnss)
