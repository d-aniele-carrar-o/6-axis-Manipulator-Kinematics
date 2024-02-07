% Direct Kinematics for IRB 7600-400/2.55, 404 mm for LeanID + draw 3D frames
% q : six joint variable angles
% pe: cartesian position of the end-effector
% Re: rotation matrix of the end-effector
% handles: handles to the different transformations. The first element links to the figure 
% firstTime
function [T06, handlesR, frame_names] = ABB_direct_kinematics_draw( q, handles, firstTime )
    % load manipulator parameters
    params
    
    % homogeneous transformation matrix from frame [i-1] to frame [i]:
    % [i-1]T[i] = Rot(x[i-1],alpha(i-1)) * Trasl(x[i-1],a[i-1]) * Rot(z[i],theta[i]) * Trasl(z[i],d[i])

    % homogeneous transformation matrix frame 0 -> 1
    T01 = @(q1) rot_tralsX( AL(1), A(1) ) * rot_traslZ( q1, D(2) );
    
    % homogeneous transformation matrix frame 1 -> 2
    T12 = @(q2) rot_tralsX( AL(2), A(2) ) * rot_traslZ( q2, D(3) );
    
    % homogeneous transformation matrix frame 2 -> 3
    T23 = @(q3) rot_tralsX( AL(3), A(3) ) * rot_traslZ( q3, D(4) );
    
    % homogeneous transformation matrix frame 3 -> 4
    T34 = @(q4) rot_tralsX( AL(4), A(4) ) * rot_traslZ( q4, D(5) );

    % homogeneous transformation matrix frame 4 -> 5
    T45 = @(q5) rot_tralsX( AL(5), A(5) ) * rot_traslZ( q5, D(6) );

    % homogeneous transformation matrix frame 5 -> 6
    T56 = @(q6) rot_tralsX( AL(6), A(6) ) * rot_traslZ( q6, D(7) );

    % check joint limits
    % q = check_limits( q, limits );

    % compute matrices with actual joint variables values q=[q1, q2, q3, q4, q5, q6]
    T01m   = T01(q(1));
    T12m   = T12(q(2));
    T23m   = T23(q(3));
    T34m_p = (rot_tralsX( AL(4), A(4) ) * rot_traslZ( q(4), D(5)/2) );
    T34m   = T34(q(4));
    T45m   = T45(q(5));
    T56m   = T56(q(6));
    
    % compute total transformation matrix frame 0 -> 6
    T06 = T01m*T12m*T23m*T34m*T45m*T56m;
    
    % extract position and orientation from homogeneous matrix
    pe = T06(1:3,4);
    % Re = T06(1:3,1:3);
    
    axs = handles(1);
    
    frame_names = zeros(8);

    % If first time draw, else update
    if firstTime
        % FRAME 0 - draw first frame (basis)
        h0 = triad( 'Parent', axs, 'linewidth', 3, 'scale', 0.3 );
        
        % FRAME 1 ===========================================================
        % transform to the end of the link and draw frame and link
        t1  = hgtransform('Parent', h0, 'Matrix', T01m);
        % triad object: composed of 3 lines representing a frame [x,y,z]
        h1  = triad('Parent', t1, 'linewidth', 3, 'scale', 0.2);
        % link representation from frame [i-1] to frame [i]
        O01 = T01m(1:3,4);
        plot3([O01(1), 0], [O01(2), 0], [O01(3), 0], 'Parent', h0, 'linestyle', '--');
        % set plot point labels for frames
        frame_names(1) = text(0.1, -0.1, 0, "Frame 0", 'FontSize', 10);
        frame_names(2) = text(O01(1)+0.1, O01(2)-0.1, O01(3), "Frame 1", 'FontSize', 10);
        % ===================================================================
        
        % FRAME 2 ===========================================================
        t2   = hgtransform('Parent', t1, 'Matrix', T12m);
        h2   = triad('Parent', t2, 'linewidth', 3, 'scale', 0.2); 
        O12  = T12m(1:3,4);
        T02m = T01m * T12m;
        O02  = T02m(1:3,4);
        plot3([O12(1), 0], [O12(2), 0], [O12(3), 0], 'Parent', h1, 'linestyle', '--');
        frame_names(3) = text(O02(1)+0.1, O02(2)-0.1, O02(3), "Frame 2", 'FontSize', 10);
        
        % FRAME 3 ===========================================================
        t3   = hgtransform('Parent', t2, 'Matrix', T23m);
        h3   = triad('Parent', t3, 'linewidth', 3, 'scale', 0.2);  
        O23  = T23m(1:3,4);
        T03m = T02m * T23m;
        O03  = T03m(1:3,4);
        plot3([O23(1), 0], [O23(2), 0], [O23(3), 0], 'Parent', h2,'linestyle', '--');
        frame_names(4) = text(O03(1)+0.1, O03(2)-0.1, O03(3), "Frame 3", 'FontSize', 10);
        
        % FRAME 4_p =========================================================
        t4_p = hgtransform('Parent', t3, 'Matrix', T34m_p);
        h4_p = triad('Parent', t4_p, 'linewidth', 3, 'scale', 0.2);
        T04m_p = T03m * T34m_p;
        O04_p = T04m_p(1:3,4);
        
        % FRAME 4 ===========================================================
        t4   = hgtransform('Parent', t3, 'Matrix', T34m);
        h4   = triad('Parent', t4, 'linewidth', 1, 'scale', 0.2);  
        Op34 = [A(4), 0, 0];
        O34  = T34m(1:3,4);
        T04m = T03m * T34m;
        O04  = T04m(1:3,4);
        plot3([Op34(1), 0], [Op34(2), 0], [Op34(3), 0], 'Parent', h3, 'linestyle','--')
        plot3([O34(1), Op34(1)], [O34(2), Op34(2)], [O34(3), Op34(3)], 'Parent', h3, 'linestyle','--')
        frame_names(5) = text(O04_p(1)+0.1, O04_p(2)-0.1, O04_p(3), "Frame 4'", 'FontSize', 10);
        frame_names(6) = text(O04(1)+0.1, O04(2)-0.1, O04(3), "Frame 4", 'FontSize', 10);
        
        % FRAME 5 ===========================================================
        t5 = hgtransform('Parent', t4, 'Matrix', T45m);
        h5 = triad('Parent', t5, 'linewidth', 3, 'scale', 0.2);  
        O45 = T45m(1:3, 4);
        T05 = T04m * T45m;
        O05 = T05(1:3,4);
        plot3([O45(1), 0], [O45(2), 0], [O45(3), 0], 'Parent', h4,'linestyle','--')
        frame_names(7) = text(O05(1)+0.1, O05(2)+0.1, O05(3)+0.2, "Frame 5", 'FontSize', 10);

        % FRAME 6 ===========================================================
        t6 = hgtransform('Parent', t5, 'Matrix', T56m);
        h6 = triad('Parent', t6, 'linewidth', 5, 'scale', 0.3); 
        O56 = T56m(1:3,4);
        plot3([O56(1), 0], [O56(2), 0], [O56(3), 0], 'Parent', h5,'linestyle','--')
        frame_names(8) = text(pe(1)+0.1, pe(2)-0.1, pe(3), "Frame 6", 'FontSize', 10);
    else
        t1   = handles(2);
        t2   = handles(3);
        t3   = handles(4);
        t4_p = handles(5);
        t4   = handles(6);
        t5   = handles(7);
        t6   = handles(8);
        
        set(t1,   'Matrix', T01m);
        set(t2,   'Matrix', T12m);
        set(t3,   'Matrix', T23m);
        set(t4_p, 'Matrix', T34m_p)
        set(t4,   'Matrix', T34m);
        set(t5,   'Matrix', T45m);
        set(t6,   'Matrix', T56m);
        
        drawnow;

        % update frame name positions
        O01 = T01m(1:3,4);
        frame_names(1) = text(0.1, -0.1, 0, "Frame 0", 'FontSize', 10);
        frame_names(2) = text(O01(1)+0.1, O01(2)-0.1, O01(3), "Frame 1", 'FontSize', 10);
        T02m = T01m * T12m;
        O02  = T02m(1:3,4);
        frame_names(3) = text(O02(1)+0.1, O02(2)-0.1, O02(3), "Frame 2", 'FontSize', 10);
        T03m = T02m * T23m;
        O03  = T03m(1:3,4);
        frame_names(4) = text(O03(1)+0.1, O03(2)-0.1, O03(3), "Frame 3", 'FontSize', 10);
        T04m_p = T03m * T34m_p;
        O04_p = T04m_p(1:3,4);
        frame_names(5) = text(O04_p(1)+0.1, O04_p(2)-0.1, O04_p(3), "Frame 4'", 'FontSize', 10);
        T04m = T03m * T34m;
        O04  = T04m(1:3,4);
        frame_names(6) = text(O04(1)+0.1, O04(2)-0.1, O04(3)-0.2, "Frame 4", 'FontSize', 10);
        T05 = T04m * T45m;
        O05 = T05(1:3,4);
        frame_names(7) = text(O05(1)+0.1, O05(2)+0.1, O05(3)+0.2, "Frame 5", 'FontSize', 10);
        frame_names(8) = text(pe(1)+0.1, pe(2)-0.1, pe(3), "Frame 6", 'FontSize', 10);
    end

    handlesR(1) = axs;
    handlesR(2) = t1;
    handlesR(3) = t2;
    handlesR(4) = t3;
    handlesR(5) = t4_p;
    handlesR(6) = t4;
    handlesR(7) = t5;
    handlesR(8) = t6;
end
