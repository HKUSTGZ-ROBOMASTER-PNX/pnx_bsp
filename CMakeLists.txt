cmake_minimum_required(VERSION 3.22)

add_library(bsp INTERFACE)
target_compile_features(bsp INTERFACE cxx_std_17)
target_link_libraries(bsp INTERFACE board libs)

file(GLOB _bsp_include_dirs CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/*/include")
target_include_directories(bsp INTERFACE ${_bsp_include_dirs})

file(GLOB _bsp_files CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*/include/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/*/src/*.cpp"
)
list(FILTER _bsp_files EXCLUDE REGEX "/can/src/(bxcan|fdcan)[.]cpp$")

if(BOARD STREQUAL "h723_mc02")
    set(_bsp_can fdcan)
elseif(BOARD STREQUAL "f407_cboard")
    set(_bsp_can bxcan)
else()
    message(FATAL_ERROR "No CAN implementation selected for BOARD '${BOARD}'")
endif()
target_sources(bsp INTERFACE ${_bsp_files} "${CMAKE_CURRENT_SOURCE_DIR}/can/src/${_bsp_can}.cpp")

# Replace generated application hooks; retain descriptors, HAL and middleware.
get_target_property(_firmware_sources ${CMAKE_PROJECT_NAME} SOURCES)
list(FILTER _firmware_sources EXCLUDE REGEX
    "/(AZURE_RTOS/App/app_azure_rtos|Core/Src/app_threadx|USBX/App/(app_usbx_device|ux_device_cdc_acm))[.]c$"
)
set_property(TARGET ${CMAKE_PROJECT_NAME} PROPERTY SOURCES "${_firmware_sources}")
