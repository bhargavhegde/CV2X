# Interface library to use the static ASN.1 lib as an existing CMake target
add_library (cms_v2x_asn-stat INTERFACE)
target_include_directories (cms_v2x_asn-stat INTERFACE ${CMS_V2X_SDK_DIR}/include)
target_include_directories (cms_v2x_asn-stat INTERFACE ${CMS_V2X_SDK_DIR}/include/asn1)
target_compile_options (cms_v2x_asn-stat INTERFACE ${CMS_V2X_SDK_C_FLAGS})
target_link_libraries (cms_v2x_asn-stat INTERFACE ${CMS_V2X_SDK_DIR}/lib/libits-asnsdk.a)

# Interface library to use the shared ASN.1 lib as an existing CMake target
add_library (cms_v2x_asn-shr INTERFACE)
target_include_directories (cms_v2x_asn-shr INTERFACE ${CMS_V2X_SDK_DIR}/include)
target_include_directories (cms_v2x_asn-shr INTERFACE ${CMS_V2X_SDK_DIR}/include/asn1)
target_compile_options (cms_v2x_asn-shr INTERFACE ${CMS_V2X_SDK_C_FLAGS})
target_link_libraries (cms_v2x_asn-shr INTERFACE ${CMS_V2X_SDK_DIR}/lib/libits-asnsdk.so)
