clc;

params

mex -setup C++

% mex src/transf_i_1_i_cpp.cpp -outdir mex_compiled_functions/
% mex src/direct_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex src/direct_kinematics_complete_cpp.cpp -outdir mex_compiled_functions/
% mex src/direct_kinematics_complete_zero_cpp.cpp -outdir mex_compiled_functions/
% mex src/UR5_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex src/custom_manipulator_inverse_kinematics_cpp.cpp -outdir mex_compiled_functions/
% mex src/Jacobian_cpp.cpp -outdir mex_compiled_functions/
% mex src/inverse_differential_kinematics_cpp.cpp -outdir mex_compiled_functions/
