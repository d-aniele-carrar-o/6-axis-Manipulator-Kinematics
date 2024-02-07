% Simulates using Inverse Differential Kineamtics and Control
% A:parameters
% Jac: jacobian (function of q)
% xd: desired position pf the of end effector (function of time)
% TH0: initial position of the joints
% minT, maxT: minimum and maximum time
% K: positive definite matrix used for control
% Dt: delta time
function [TH] = invDiffKinematicControlSimComplete(direct, Jac, xd, phid, TH0, Kp, Kphi, minT, maxT, Dt)

    T  = [minT:Dt:maxT];
    L  = length(T);
    qk = [TH0];
    q  = [qk];
    for t = T(2:L)
        fprintf("invDiffKin - t=%.4f\n", t)
        Te = direct(qk);
        xe = Te(1:3,4);
        Re = Te(1:3,1:3);
        phie = rotm2eulFDR(Re);
        vd = (xd(t)-xd(t-Dt))/Dt;
        phiddot = (phid(t)-phid(t-Dt))/Dt;
        
        % computed joint-space velocity corresponding to the desired task-space velocity
        dotqk = invDiffKinematiControlComplete(Jac,qk, xe, xd(t), vd, phie', phid(t),phiddot, Kp, Kphi);
        
        % euler integration step
        qk1 = qk + dotqk'*Dt;
        q = [q; qk1];
        qk = qk1;
    end
    TH = q;
end