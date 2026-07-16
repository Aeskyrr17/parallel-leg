cmake_minimum_required(VERSION 3.22)

if(NOT CMAKE_TOOLCHAIN_FILE)
    set(PNX_EMBEDDED_COMPILER_HINT "${CMAKE_C_COMPILER};${CMAKE_CXX_COMPILER}")
    if(PNX_EMBEDDED_COMPILER_HINT MATCHES "arm-none-eabi")
        include("${CMAKE_CURRENT_SOURCE_DIR}/board/cmake/gcc-arm-none-eabi.cmake")
    endif()
    unset(PNX_EMBEDDED_COMPILER_HINT)
endif()

project(pnx_embedded LANGUAGES C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_C_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(PNX_EMBEDDED_CONFIG_DIR "${CMAKE_CURRENT_SOURCE_DIR}/configs")
set(PNX_EMBEDDED_GENERATED_DIR "${PNX_EMBEDDED_CONFIG_DIR}/generated")
set(PNX_EMBEDDED_ROBOT_CONFIG "${PNX_EMBEDDED_CONFIG_DIR}/robot.json")

set_property(DIRECTORY APPEND PROPERTY CMAKE_CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/board/board.ioc"
    "${PNX_EMBEDDED_CONFIG_DIR}/params.json"
    "${PNX_EMBEDDED_ROBOT_CONFIG}"
    "${PNX_EMBEDDED_CONFIG_DIR}/cmake/generate_config.cmake"
    "${PNX_EMBEDDED_CONFIG_DIR}/cmake/import_ioc.cmake"
)

set(IOC "${CMAKE_CURRENT_SOURCE_DIR}/board/board.ioc")
set(PARAMS "${PNX_EMBEDDED_CONFIG_DIR}/params.json")
set(ROBOT_CONFIG "${PNX_EMBEDDED_ROBOT_CONFIG}")
set(OUT_DIR "${PNX_EMBEDDED_GENERATED_DIR}")
include("${PNX_EMBEDDED_CONFIG_DIR}/cmake/generate_config.cmake")
unset(IOC)
unset(PARAMS)
unset(ROBOT_CONFIG)
unset(OUT_DIR)

file(GLOB_RECURSE PNX_EMBEDDED_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/demo/*.cpp"
)

if(NOT ENABLE_USBX)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]pnx_bsp[/\\\\]usb[/\\\\]")
endif()

file(GLOB_RECURSE PNX_EMBEDDED_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/demo/*.hpp"
)

set(PNX_EMBEDDED_INCLUDE_DIRS)
foreach(header IN LISTS PNX_EMBEDDED_HEADERS)
    get_filename_component(header_dir "${header}" DIRECTORY)
    list(APPEND PNX_EMBEDDED_INCLUDE_DIRS "${header_dir}")
endforeach()
list(REMOVE_DUPLICATES PNX_EMBEDDED_INCLUDE_DIRS)

set(CMSIS_DSP_SOURCES
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/BasicMathFunctions/arm_dot_prod_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_add_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_init_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_inverse_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_mult_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_sub_f32.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Source/MatrixFunctions/arm_mat_trans_f32.c"
)

add_executable(${PROJECT_NAME})

add_subdirectory(board/cmake/stm32cubemx)

target_sources(${PROJECT_NAME} PRIVATE
    ${PNX_EMBEDDED_SOURCES}
    ${CMSIS_DSP_SOURCES}
)

target_include_directories(${PROJECT_NAME} PRIVATE
    ${PNX_EMBEDDED_INCLUDE_DIRS}
    "${CMAKE_CURRENT_SOURCE_DIR}/cmsis-dsp/Include"
    "${PNX_EMBEDDED_GENERATED_DIR}"
)

target_link_libraries(${PROJECT_NAME}
    stm32cubemx
)

list(REMOVE_ITEM CMAKE_C_IMPLICIT_LINK_LIBRARIES ob)
