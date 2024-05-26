# CMake file to compile Commsignia V2X Remote C SDK examples

message (STATUS "Building Commsignia V2X Remote C SDK examples")

if (NOT DEFINED CMS_V2X_EXAMPLES_DIR)
    message (FATAL_ERROR "Please define CMS_V2X_EXAMPLES_DIR")
endif ()


# Select to link to the static or shared lib
if (CMS_V2X_USE_STATIC)
    message (STATUS "  linked with the static SDK lib")
    set (CMS_V2X_SDK cms_v2x_sdk-stat)
    set (CMS_V2X_ASN_SDK cms_v2x_asn-stat)
else ()
    message (STATUS "  linked with the shared SDK lib")
    set (CMS_V2X_SDK cms_v2x_sdk-shr)
    set (CMS_V2X_ASN_SDK cms_v2x_asn-shr)
endif ()


# List all examples and create targets
# - CMS SDK examples
file(GLOB CMS_V2X_EXAMPLE_SRCS LIST_DIRECTORIES false ${CMS_V2X_EXAMPLES_DIR}/*.c)

foreach(EXAMPLE_FILE ${CMS_V2X_EXAMPLE_SRCS})

    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    message (STATUS "  Adding example ${EXAMPLE_NAME}")
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE ${CMS_V2X_SDK})
    # some functions used in examples, like usleep and clock_gettime, require GNU extension from libc
    target_compile_definitions(${EXAMPLE_NAME} PRIVATE _GNU_SOURCE=1)

endforeach()

# - ASN.1 examples
file(GLOB CMS_V2X_ASN_EXAMPLE_SRCS LIST_DIRECTORIES false ${CMS_V2X_EXAMPLES_DIR}/asn/*.c)

foreach(EXAMPLE_FILE ${CMS_V2X_ASN_EXAMPLE_SRCS})

    get_filename_component(EXAMPLE_NAME ${EXAMPLE_FILE} NAME_WE)
    message (STATUS "  Adding example ${EXAMPLE_NAME}")
    add_executable(${EXAMPLE_NAME} ${EXAMPLE_FILE})
    target_link_libraries(${EXAMPLE_NAME} PRIVATE ${CMS_V2X_SDK} ${CMS_V2X_ASN_SDK})

endforeach()
