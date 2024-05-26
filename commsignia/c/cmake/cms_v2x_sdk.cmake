# CMake file to provide 'cms_v2x_sdk-stat' target interface for
# Commsignia V2X Remote C SDK
# Provided as an example to integrate the SDK into a cmake project.

message (STATUS "Using Commsignia V2X Remote C SDK")

if (NOT DEFINED CMS_V2X_SDK_DIR)
    message (FATAL_ERROR "CMS_V2X_SDK_DIR is not defined")
endif ()

list (APPEND CMS_V2X_SDK_C_FLAGS -std=c11 -O2 -g -fPIC)
list (APPEND CMS_V2X_SDK_C_FLAGS -Wall -Wno-unused-variable -Wno-unused-local-typedefs)

# Interface library to use the static SDK lib as an existing CMake target
add_library (cms_v2x_sdk-stat INTERFACE)
target_include_directories (cms_v2x_sdk-stat INTERFACE ${CMS_V2X_SDK_DIR}/include)
target_compile_options (cms_v2x_sdk-stat INTERFACE ${CMS_V2X_SDK_C_FLAGS})
target_link_libraries (cms_v2x_sdk-stat INTERFACE ${CMS_V2X_SDK_DIR}/lib/libits-rem.a)
target_link_libraries (cms_v2x_sdk-stat INTERFACE stdc++)

# Interface library to use the shared SDK lib as an existing CMake target
add_library (cms_v2x_sdk-shr INTERFACE)
target_include_directories (cms_v2x_sdk-shr INTERFACE ${CMS_V2X_SDK_DIR}/include)
target_compile_options (cms_v2x_sdk-shr INTERFACE ${CMS_V2X_SDK_C_FLAGS})
target_link_libraries (cms_v2x_sdk-shr INTERFACE ${CMS_V2X_SDK_DIR}/lib/libits-rem.so)
