% Moves from an initial pose to a target frame representing an object
clc;
clear all;
close all;

Samples = 50;
global Tm
global xe0;
global xef;
global phie0;
global phief;

Tm = 1;
Tf = Tm;
T  = linspace(0,Tf,Samples);
Th = [];

%Initial pose
xe0   = [ 0.3; 0.3; 0.4];
xef   = [-0.4; 0.2; 0.6];
phie0 = [   0;   0;   0];
phief = [   0;   0;  pi]; 

% we assume ZYX convention but the vector is stored as X,Y,Z coordinates, 
% so [0,0,pi] is a rotation of pi about Z axis

DeltaT = Tf/Samples ;  
TH0 = UR5_inverse_kinematics(pd(0), eul2rotm(phid(0)));

Kp = 5*eye(3,3);
%Kphi = diag([0.1,0.1,0.5]);
Kphi = 0.1*eye(3,3);
fprintf("invDiffKinematicControlSimComplete - start")
[Th]= invDiffKinematicControlSimComplete(@UR5_direct_kinematics, @Jacobian, @pd, @phid, TH0(1,:), Kp, Kphi, 0, Tf, DeltaT);
fprintf("invDiffKinematicControlSimComplete - end")

lim = 0.6;
scaleFactor = 1;
limS = scaleFactor*lim;
axs=axes('XLim',[-limS limS],'YLim',[-limS limS],'ZLim',[-limS limS]); view(3); grid on;
xlabel(['X x ', num2str(scaleFactor)], 'FontSize',12);
ylabel(['Y x ', num2str(scaleFactor)], 'FontSize',12);
zlabel(['Z x ', num2str(scaleFactor)], 'FontSize',12);
handles(1) = axs;

[Te, handlesR] = UR5_direct_kinematics_draw(Th(1,:), handles, true);
pe = Te(1:3,4);
Re = Te(1:3,1:3);

Tt = [eul2rotm(phief), xef*scaleFactor; zeros(1,3), 1];
tt = hgtransform('Parent', axs, 'Matrix', Tt);
ht = triad('Parent', tt, 'linewidth', 4, 'scale', 0.2);
peA = pe;
phieA = rotm2eul(Re)';

for i = 2:max(size(Th))
    [Te, handlesR] = UR5_direct_kinematics_draw(Th(i,:), handlesR, false);
    pe = Te(1:3,4);
    Re = Te(1:3,1:3);
    peA = [peA pe];
    phieA = [phieA  rotm2eulFDR(Re)'];
    pause(0.00001)
end

% plot trajectory in 3D space
Samples = max(size(peA));
figure;
lim = scaleFactor*lim;           
axs = axes('XLim', [-lim lim], 'YLim', [-lim lim], 'ZLim', [-lim lim]); view(3); grid on;
xlabel('X', 'FontSize',12);ylabel('Y', 'FontSize',12);zlabel('Z', 'FontSize',12); hold on;

for i = 1:Samples-1
   plot3([peA(1,i), peA(1,i+1)], [peA(2,i), peA(2,i+1)], [peA(3,i), peA(3,i+1)], 'Parent',axs);
end
    


% position interpolation
function [xd] = pd(t)
global xef; global xe0;
global Tm;
    if (t > Tm)
        xd = xef;
    else
        xd = (t/Tm)*xef + (1-(t/Tm))*xe0;
    end
end

% orientation interpolation
function [phid] = phid(t)
global phief; global phie0;
global Tm;
    if (t > Tm)
        phid = phief;
    else
        phid = (t/Tm)*phief + (1-(t/Tm))*phie0;
    end
end

