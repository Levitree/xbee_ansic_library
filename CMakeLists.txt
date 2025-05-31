cmake_minimum_required(VERSION 3.16)
    
file(GLOB_RECURSE NESTED_SRC
       src/**.c
       ports/idf/**.c
)

list(APPEND INCLUDE_DIRS "include")
list(APPEND INCLUDE_DIRS "ports/idf")

idf_component_register(
    SRCS ${NESTED_SRC}
    INCLUDE_DIRS ${INCLUDE_DIRS}
    REQUIRES esp_hw_support
    PRIV_REQUIRES esp_driver_uart esp_driver_gpio mbedtls
)

idf_build_set_property(COMPILE_OPTIONS "-DXBEE_PLATFORM_HEADER=\"platform_config.h\"" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DXBEE_WIFI_ENABLED" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-DXBEE_CELLULAR_ENABLED" APPEND)

idf_build_set_property(COMPILE_OPTIONS "-Wno-error=maybe-uninitialized" APPEND)
idf_build_set_property(COMPILE_OPTIONS "-Wno-error=format-overflow" APPEND)