
cmake_minimum_required(VERSION 3.22)

add_library(dsp STATIC
    src/BasicMathFunctions/arm_dot_prod_f32.c
    src/MatrixFunctions/arm_mat_add_f32.c
    src/MatrixFunctions/arm_mat_init_f32.c
    src/MatrixFunctions/arm_mat_inverse_f32.c
    src/MatrixFunctions/arm_mat_mult_f32.c
    src/MatrixFunctions/arm_mat_sub_f32.c
    src/MatrixFunctions/arm_mat_trans_f32.c
)

target_include_directories(dsp PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_compile_features(dsp PUBLIC c_std_11)

# The selected board supplies CMSIS Core headers and device definitions.
target_link_libraries(dsp PUBLIC stm32cubemx)
