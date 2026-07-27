# Include this file from the firmware CMakeLists.txt.
#
# Before use, set:
#   LQR_TOOL_DIR       directory containing these Python files
#   LQR_COEFFS_HPP     destination control/include/lqr_coeffs.hpp
#   FIRMWARE_TARGET    actual firmware target name

find_package(Python3 REQUIRED COMPONENTS Interpreter)

set(LQR_GENERATOR "${LQR_TOOL_DIR}/generate_lqr_header.py")

add_custom_command(
    OUTPUT "${LQR_COEFFS_HPP}"
    COMMAND "${Python3_EXECUTABLE}" "${LQR_GENERATOR}"
            --output "${LQR_COEFFS_HPP}"
    DEPENDS
        "${LQR_GENERATOR}"
        "${LQR_TOOL_DIR}/fit_lqr.py"
        "${LQR_TOOL_DIR}/dynamics.py"
        "${LQR_TOOL_DIR}/amatrix.py"
        "${LQR_TOOL_DIR}/bmatrix.py"
        "${LQR_TOOL_DIR}/model.py"
        "${LQR_TOOL_DIR}/leg_data.py"
    WORKING_DIRECTORY "${LQR_TOOL_DIR}"
    COMMENT "Generating parallel-leg LQR coefficients"
    VERBATIM
)

add_custom_target(generate_lqr_coeffs DEPENDS "${LQR_COEFFS_HPP}")
add_dependencies("${FIRMWARE_TARGET}" generate_lqr_coeffs)
