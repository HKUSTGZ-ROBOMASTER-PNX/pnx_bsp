cmake_minimum_required(VERSION 3.22)

add_library(libs INTERFACE)
target_compile_features(libs INTERFACE cxx_std_17)
target_include_directories(libs INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/crc/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/filters/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/math/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/msg/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/types/include"
)

# boards supplies the selected board's ThreadX headers and configuration.
target_link_libraries(libs INTERFACE stm32cubemx)
