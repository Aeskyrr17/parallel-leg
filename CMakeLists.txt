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

option(PNX_AUTO_GENERATE_LQR
    "Regenerate application/leg/Inc/lqr.hpp from the MATLAB LQR parameters"
    ON
)

if(PNX_AUTO_GENERATE_LQR)
    find_program(PNX_MATLAB_EXECUTABLE NAMES matlab matlab.exe)
    if(NOT PNX_MATLAB_EXECUTABLE)
        message(FATAL_ERROR
            "MATLAB was not found. Install MATLAB or configure with "
            "-DPNX_AUTO_GENERATE_LQR=OFF to use the coefficients already in lqr.hpp."
        )
    endif()

    set(PNX_LQR_MATLAB_DIR
        "${CMAKE_CURRENT_SOURCE_DIR}/application/leg/Matlab"
    )
    set(PNX_LQR_CONFIG "${PNX_LQR_MATLAB_DIR}/lqr_config.m")
    set(PNX_LQR_GENERATOR "${PNX_LQR_MATLAB_DIR}/lqr_tuner.m")
    set(PNX_LQR_HEADER
        "${CMAKE_CURRENT_SOURCE_DIR}/application/leg/Inc/lqr.hpp"
    )
    set(PNX_LQR_STAMP
        "${CMAKE_CURRENT_BINARY_DIR}/generated/lqr_coefficients.stamp"
    )

    file(TO_CMAKE_PATH "${PNX_LQR_MATLAB_DIR}" PNX_LQR_MATLAB_DIR_ARG)
    file(TO_CMAKE_PATH "${PNX_LQR_HEADER}" PNX_LQR_HEADER_ARG)
    set(PNX_LQR_BATCH_COMMAND
        "addpath('${PNX_LQR_MATLAB_DIR_ARG}'); lqr_tuner(true, '${PNX_LQR_HEADER_ARG}');"
    )

    add_custom_command(
        OUTPUT "${PNX_LQR_STAMP}"
        BYPRODUCTS "${PNX_LQR_HEADER}"
        COMMAND "${CMAKE_COMMAND}" -E make_directory
                "${CMAKE_CURRENT_BINARY_DIR}/generated"
        COMMAND "${PNX_MATLAB_EXECUTABLE}" -batch "${PNX_LQR_BATCH_COMMAND}"
        COMMAND "${CMAKE_COMMAND}" -E touch "${PNX_LQR_STAMP}"
        DEPENDS
            "${PNX_LQR_CONFIG}"
            "${PNX_LQR_GENERATOR}"
            "${CMAKE_CURRENT_SOURCE_DIR}/application/leg/Inc/leg_config.hpp"
        COMMENT "Generating LQR coefficients from application/leg/Matlab/lqr_config.m"
        VERBATIM
    )
    add_custom_target(pnx_lqr_coefficients DEPENDS "${PNX_LQR_STAMP}")
endif()

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
    "${CMAKE_CURRENT_SOURCE_DIR}/application/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/application/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.cpp"
)

if(NOT ENABLE_USBX)
    list(FILTER PNX_EMBEDDED_SOURCES EXCLUDE REGEX "[/\\\\]pnx_bsp[/\\\\]usb[/\\\\]")
endif()

file(GLOB_RECURSE PNX_EMBEDDED_HEADERS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/application/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/application/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_bsp/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_devices/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_modules/*.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.h"
    "${CMAKE_CURRENT_SOURCE_DIR}/pnx_libs/*.hpp"
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

if(PNX_AUTO_GENERATE_LQR)
    add_dependencies(${PROJECT_NAME} pnx_lqr_coefficients)
endif()

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
