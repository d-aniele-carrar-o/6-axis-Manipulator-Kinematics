clc;
close all;

%%
% load parameters
params

% add path to utils folder
addpath( "ABB_scripts/" )
addpath( "utils/" )

% run( "testing/testing_script.m" )

scaleFactor = 1;

% if true, compute AND plot the manipulator configuration
draw = true;

lim = 1;
alfa = 340;
beta = 140;
axs  = axes( 'XLim', [-0.5, 0.5], 'YLim', [-0.5, 0.5], 'ZLim', [-0.1, 0.7] );
view( alfa, beta ); grid on;
handles(1) = axs;




% joint angles configuration
q = [0.33, 2.476, -1.189, 2.127, 0.563, -2.138]

% compute direct kinematics for defined joint angles q
T06 = ABB_direct_kinematics( q )

% compute 8 (at most) inverse kinematics solutions for given Frame_6 pose
H = ABB_inverse_kinematics( T06 )

% for every inverse kinematics solution, compute direct kinematics and draw the manipulator pose
for i=1:8
    i
    q_i = H(i,:);
    if anynan(q_i)
        fprintf("configuration impossible, moving to next one.")
        continue
    end
    if draw
        if i==1
            [T06, handlesR, frame_names] = ABB_direct_kinematics_draw( q_i, handles, true );
        else
            [T06, handlesR, frame_names] = ABB_direct_kinematics_draw( q_i, handlesR, false );
            T06
        end
        pause()
        
        % clear frame names labels
        if i < 8
            delete(frame_names)
        end
    else
        T06 = ABB_direct_kinematics( q_i )
    end
end


