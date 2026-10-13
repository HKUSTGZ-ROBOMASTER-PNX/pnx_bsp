cmake_minimum_required(VERSION 3.22)

set(_wrapper_dir "${CMAKE_CURRENT_SOURCE_DIR}/${BOARD}")
if(NOT IS_DIRECTORY "${_wrapper_dir}/include" OR NOT EXISTS "${_wrapper_dir}/src/init.cpp")
    message(FATAL_ERROR "Missing board wrapper for BOARD '${BOARD}': ${_wrapper_dir}")
endif()

add_library(board INTERFACE)
target_compile_features(board INTERFACE cxx_std_17)
target_include_directories(board INTERFACE "${_wrapper_dir}/include")
target_link_libraries(board INTERFACE stm32cubemx)

file(GLOB _wrapper_headers CONFIGURE_DEPENDS "${_wrapper_dir}/include/*.hpp")
target_sources(board INTERFACE ${_wrapper_headers})

# main() belongs to the firmware, not to every consumer of the BSP interface.
target_sources(${CMAKE_PROJECT_NAME} PRIVATE "${_wrapper_dir}/src/init.cpp")
