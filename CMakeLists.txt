cmake_minimum_required(VERSION 3.22)

if(NOT DEFINED PNX_CONFIGS_DIR)
    set(PNX_CONFIGS_DIR ${CMAKE_SOURCE_DIR}/pnx_configs)
endif()

set(BSP_MODULE_DIRS adc can dwt iic pwm spi usart ws2812)

set(BSP_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/src/bsp.cpp)

foreach(module ${BSP_MODULE_DIRS})
    file(GLOB module_sources ${CMAKE_CURRENT_SOURCE_DIR}/${module}/src/*.cpp)
    list(APPEND BSP_SOURCES ${module_sources})
endforeach()

add_library(pnx_bsp STATIC ${BSP_SOURCES})

set(BSP_PUBLIC_INCLUDES
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PNX_CONFIGS_DIR}
)

foreach(module ${BSP_MODULE_DIRS})
    list(APPEND BSP_PUBLIC_INCLUDES ${CMAKE_CURRENT_SOURCE_DIR}/${module}/include)
endforeach()

target_include_directories(pnx_bsp PUBLIC ${BSP_PUBLIC_INCLUDES})

target_link_libraries(pnx_bsp PUBLIC pnx_os pnx_platform)

target_compile_features(pnx_bsp PUBLIC cxx_std_17)
