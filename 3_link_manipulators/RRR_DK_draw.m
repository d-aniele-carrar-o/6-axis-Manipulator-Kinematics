% Direct Kinematics for IRB 7600-400/2.55, 404 mm for LeanID + draw 3D frames
% q : six joint variable angles
% pe: cartesian position of the end-effector
% Re: rotation matrix of the end-effector
% handles: handles to the different transformations. The first element links to the figure 
% firstTime
function [T04, handlesR] = RRR_DK_draw( q, handles, firstTime )
    RRR_params

    T01 = RRR_trasf_i_1_i( 1, q(1) );
    T12 = RRR_trasf_i_1_i( 2, q(2) );
    T23 = RRR_trasf_i_1_i( 3, q(3) );
    T34 = RRR_trasf_i_1_i( 4, 0 );

    T04 = T01*T12*T23*T34;

    axs = handles(1);

    % If first time draw, else update
    if firstTime
        % FRAME 0 - draw first frame (basis)
        h0 = triad( 'Parent', axs, 'linewidth', 3, 'scale', 0.15 );
        
        % FRAME 1 ===========================================================
        % transform to the end of the link and draw frame and link
        t1  = hgtransform('Parent', h0, 'Matrix', T01);
        % triad object: composed of 3 lines representing a frame [x,y,z]
        h1  = triad('Parent', t1, 'linewidth', 2, 'scale', 0.1);
        % link representation from frame [i-1] to frame [i]
        O01 = T01(1:3,4);
        plot3([O01(1), 0], [O01(2), 0], [O01(3), 0], 'Parent', h0, 'linestyle', '--');
        % ===================================================================
        
        % FRAME 2 ===========================================================
        t2   = hgtransform('Parent', t1, 'Matrix', T12);
        h2   = triad('Parent', t2, 'linewidth', 2, 'scale', 0.1); 
        O12  = T12(1:3,4);
        T02  = T01 * T12;
        plot3([O12(1), 0], [O12(2), 0], [O12(3), 0], 'Parent', h1, 'linestyle', '--');
        
        % FRAME 3 ===========================================================
        t3   = hgtransform('Parent', t2, 'Matrix', T23);
        h3   = triad('Parent', t3, 'linewidth', 2, 'scale', 0.1);
        O23p = [A(3), 0, 0];
        O23  = T23(1:3,4);
        T03  = T02 * T23;
        plot3([O23p(1), 0], [O23p(2), 0], [O23p(3), 0], 'Parent', t2, 'linestyle', '--');
        plot3([O23p(1), O23(1)], [O23p(2), O23(2)], [O23p(3), O23(3)], 'Parent', t2, 'linestyle', '--');
        
        % FRAME 4 ===========================================================
        t4   = hgtransform('Parent', t3, 'Matrix', T34);
        h4   = triad('Parent', t4, 'linewidth', 3, 'scale', 0.15);  
        O34  = T34(1:3,4);
        plot3([O34(1), 0], [O34(2), 0], [O34(3), 0], 'Parent', h3, 'linestyle','--')
        plot3([O34(1), O34(1)], [O34(2), O34(2)], [O34(3), O34(3)], 'Parent', h3, 'linestyle','--')

    else
        t1   = handles(2);
        t2   = handles(3);
        t3   = handles(4);
        t4   = handles(5);

        set(t1, 'Matrix', T01);
        set(t2, 'Matrix', T12);
        set(t3, 'Matrix', T23);
        set(t4, 'Matrix', T34);
        
        drawnow;
    end

    handlesR(1) = axs;
    handlesR(2) = t1;
    handlesR(3) = t2;
    handlesR(4) = t3;
    handlesR(5) = t4;

end
