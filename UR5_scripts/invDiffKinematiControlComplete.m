% Computation of the controlled values for the complete motion 
% Parameters:
%  Jac: jacobian (function of q)
%    q: position of the joints
%   xe: position of the end effector
%   xd: desired position of the end effector
%   vd: desired velocity fo the end effector
% phie: RPY (X-Y-Z rotation) int the fixed axis 
%           phie(1) -> X
%           phie(2) -> Y
%           phie(3) -> Z
%  phid: Desired RPY
%     K: Positive definite matrix to reduce the error
% q_dot: velocity to be appleid at the joints
function [q_dot] = invDiffKinematiControlComplete(Jac,q, xe, xd, vd, phie, phid, phiddot, Kp, Kphi)
    [J]   = Jac(q);
    psi   = phie(1);
    theta = phie(2);
    phi   = phie(3);
    
    T = [cos(theta)*cos(phi), -sin(phi), 0;
         cos(theta)*sin(phi),  cos(phi), 0;
                 -sin(theta),         0, 1];

    Ta = [  eye(3,3), zeros(3,3);
          zeros(3,3),          T];
    
    Ja = inv(Ta)*J;
    
    q_dot = inv(Ja + eye(6)*1e-06) * [(vd + Kp*(xd-xe)); (phiddot + Kphi*(phid-phie))];
    
    % to debug    
    disp('err pos')
    norm(xd-xe)
    disp('err orient')
    norm(phid-phie)
    disp('max q')
    max(abs(q_dot))
end
