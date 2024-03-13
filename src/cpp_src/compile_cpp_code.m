clc;

params

mex -setup C++

mex cpp_src/transf_i_1_i_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/direct_kinematics_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/direct_kinematics_complete_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/direct_kinematics_complete_zero_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/UR5_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/custom_manipulator_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/Jacobian_cpp.cpp -outdir mex_compiled_functions/
mex cpp_src/inverse_differential_kinematics_cpp.cpp -outdir mex_compiled_functions/
