# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

set(EXTRA_COMPONENT_DIRS components/)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Smart-glove-fw)

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wno-format -Wno-missing-field-initializers")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++20 -Wno-format -Wno-missing-field-initializers -Wno-write-strings -Wno-maybe-uninitialized")

#target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-format -Wno-missing-field-initializers -Wno-write-strings -fexceptions -Wno-maybe-uninitialized)