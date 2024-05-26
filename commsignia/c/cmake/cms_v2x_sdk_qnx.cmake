# Cross compilation for QNX
set(CMAKE_SYSTEM_NAME QNX)
set(QNX YES)

set(CMAKE_CXX_FLAGS "${CMAKE_EXE_LINKER_FLAGS} -lang-c++ -std=gnu++11")

# Add definitions command for diffent platforms So we can use macro definition
# "#ifdef PLATFORM_QNX" in source code
set(PLATFORM_QNX ON)
add_definitions(-DPLATFORM_QNX)
add_definitions(-D_QNX_SOURCE)
# Build crossguid library for qnx
set(GUID_LIBUUID ON)
add_definitions(-DGUID_LIBUUID)

# Set the path to the qnx host include and lib
include_directories($ENV{QNX_HOST}/usr/include)
link_directories($ENV{QNX_HOST}/usr/lib)

# Set the path to the qnx target include and lib
include_directories($ENV{QNX_TARGET}/usr/include)
link_directories($ENV{QNX_TARGET}/usr/lib)
include_directories($ENV{QNX_TARGET}/${ARCH_NAME}/usr/include)
link_directories($ENV{QNX_TARGET}/${ARCH_NAME}/lib)
link_directories($ENV{QNX_TARGET}/${ARCH_NAME}/usr/lib)
