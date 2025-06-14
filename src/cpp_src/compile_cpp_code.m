clc;

parameters(1)

mex -setup C++

mex_options = '-I/usr/include/ -I/usr/local/include/';

% mex cpp_src/transf_i_1_i_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/direct_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/direct_kinematics_complete_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/direct_kinematics_complete_zero_cpp.cpp -outdir mex_compiled_functions/
mex(mex_options, 'cpp_src/UR5_inverse_kinematics_cpp.cpp', '-outdir', 'mex_compiled_functions/');
% mex cpp_src/ABB_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/custom_manipulator_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/Jacobian_cpp.cpp -outdir mex_compiled_functions/
% mex cpp_src/inverse_differential_kinematics_cpp.cpp -outdir mex_compiled_functions/
