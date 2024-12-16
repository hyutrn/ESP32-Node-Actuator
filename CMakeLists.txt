# For more information about build system see
# https://docs.espressif.com/projects/esp-idf/en/latest/api-guides/build-system.html
# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.16)
set(CMAKE_CXX_STANDARD 17)
set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/protocol_examples_common)

set(EXTRA_COMPONENT_DIRS "lib")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(actuator_node_v1)

#target_add_binary_data(actuator_node_v1.elf "lib/mqtt_cfg/mqtt_cert_key.pem" TEXT)