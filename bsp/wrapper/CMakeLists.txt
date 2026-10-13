cmake_minimum_required(VERSION 3.22)

set(_wrapper_dir "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}")
if(NOT IS_DIRECTORY "${_wrapper_dir}/include" OR NOT EXISTS "${_wrapper_dir}/src/init.cpp")
    message(FATAL_ERROR "Missing BSP wrapper for BOARD '${BOARD}': ${_wrapper_dir}")
endif()

target_include_directories(bsp INTERFACE "${_wrapper_dir}/include")

file(GLOB _wrapper_headers CONFIGURE_DEPENDS "${_wrapper_dir}/include/*.hpp")
target_sources(bsp INTERFACE ${_wrapper_headers})

# main() belongs to the firmware, not to every consumer of the BSP interface.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE "${_wrapper_dir}/src/init.cpp")
