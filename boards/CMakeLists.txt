set(BOARD "h723_mc02" CACHE STRING "Target board")

set(_supported_boards f407_cboard h723_mc02)
set_property(CACHE BOARD PROPERTY STRINGS ${_supported_boards})

if(NOT BOARD IN_LIST _supported_boards)
    list(JOIN _supported_boards ", " _supported_boards_text)
    message(FATAL_ERROR
        "Unsupported BOARD '${BOARD}'. Supported boards: ${_supported_boards_text}"
    )
endif()

message(STATUS "Target board: ${BOARD}")

set(BOARD_LINKER_SCRIPT "${CMAKE_CURRENT_SOURCE_DIR}/memory/${BOARD}.ld")
if(NOT EXISTS "${BOARD_LINKER_SCRIPT}")
    message(FATAL_ERROR "Missing linker script: ${BOARD_LINKER_SCRIPT}")
endif()

target_link_options(${CMAKE_PROJECT_NAME} PRIVATE "LINKER:-T,${BOARD_LINKER_SCRIPT}")
set_property(TARGET ${CMAKE_PROJECT_NAME} APPEND PROPERTY LINK_DEPENDS "${BOARD_LINKER_SCRIPT}")
message(STATUS "Linker script: ${BOARD_LINKER_SCRIPT}")

add_subdirectory("cubemx/${BOARD}")
add_subdirectory(dsp)
add_subdirectory(wrapper)
