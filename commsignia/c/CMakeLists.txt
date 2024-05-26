# CMake project to compile Unplugged-RT remote SDK examples

cmake_minimum_required (VERSION 3.4)
project (commsignia-upl-remote-sdk-examples)
message (STATUS "Using CMake version ${CMAKE_VERSION}")
set (CMAKE_BUILD_TYPE Debug)
set (CMAKE_EXPORT_COMPILE_COMMANDS ON)

set (CMS_V2X_SDK_DIR ${PROJECT_SOURCE_DIR})
set (CMS_V2X_EXAMPLES_DIR ${CMS_V2X_SDK_DIR}/examples)

# Target binding
if (DEFINED TARGET_BINDING)
    include (${TARGET_BINDING})
endif ()

include (${CMS_V2X_SDK_DIR}/cmake/cms_v2x_sdk.cmake)
include (${CMS_V2X_EXAMPLES_DIR}/cms_v2x_sdk_examples.cmake)

# Use the latest asn library
include (${CMS_V2X_SDK_DIR}/cmake/cms_v2x_asn.cmake)
