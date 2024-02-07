%Direct Kinematics of the + draw UR5
%Th: six joint angles
%pe: cartesian position of the end effector
%Re: Rotation matrix of the end effecto
%handles: handles to the differetn transofmrations. THe first element links to the figure 
%firstTime
function [Te, handlesR] = UR5_direct_kinematics_draw( th, handles, firstTime )
    params
    
    T01 = transf_i_1_i( 1, th(1) );
    T12 = transf_i_1_i( 2, th(2) );
    T23 = transf_i_1_i( 3, th(3) );
    T34 = transf_i_1_i( 4, th(4) );
    T45 = transf_i_1_i( 5, th(5) );
    T56 = transf_i_1_i( 6, th(6) );
    
    T06 = T01 * T12 * T23 * T34 * T45 * T56;
    
    pe = T06(1:3,4);
    Re = T06(1:3, 1:3);
    Te = [Re, pe; 0, 0, 0, 1];

    axs = handles(1);
    % If first time draw, else update
    if firstTime
      %Draw first frame (basis)
      h0 = triad( 'Parent', axs, 'linewidth', 3, 'scale', 0.1 );
    
      %Transform to the end of the links and draw the frames and the links
      t1  = hgtransform( 'Parent', h0, 'Matrix', T01 );
      h1  = triad( 'Parent', t1, 'linewidth', 2, 'scale', 0.05 );
      O10 = T01(1:3,4);
      O00 = [0;0;0];
      plot3( [O00(1), O10(1)], [O00(2), O10(2)], [O00(3), O10(3)], 'Parent', h0, 'linestyle', '--' );
    
      
      t2 = hgtransform( 'Parent', t1, 'Matrix', T12 );
      h2 = triad( 'Parent', t2, 'linewidth', 2, 'scale', 0.05 );  
      
      O32 = T23(1:3,4);
      t3  = hgtransform( 'Parent', t2, 'Matrix', T23 );
      h3  = triad( 'Parent', t3, 'linewidth', 2, 'scale', 0.05 );  
      plot3([O32(1), 0], [O32(2), 0], [O32(3), 0], 'Parent',t2,'linestyle','--');
    
    
      t4   = hgtransform( 'Parent', t3, 'Matrix', T34 );
      h4   = triad( 'Parent', t4, 'linewidth', 2, 'scale', 0.05 );
      Op43 = [A(4), 0, 0];
      O43  = T34(1:3,4);
      plot3( [Op43(1), 0], [Op43(2), 0], [Op43(3), 0], 'Parent', t3, 'linestyle', '--' )
      plot3( [Op43(1), O43(1)], [Op43(2), O43(2)], [Op43(3), O43(3)], 'Parent', t3, 'linestyle', '--' )
    
    
      t5  = hgtransform( 'Parent', t4, 'Matrix', T45 );
      h5  = triad( 'Parent', t5, 'linewidth', 2, 'scale', 0.05 );  
      O54 = T45(1:3, 4);
      plot3( [O54(1), 0], [O54(2), 0], [O54(3), 0], 'Parent', t4, 'linestyle', '--' )
      
      t6  = hgtransform( 'Parent', t5, 'Matrix', T56 );
      h6  = triad( 'Parent', t6, 'linewidth', 3, 'scale', 0.1 ); 
      O65 = T56(1:3,4);
      plot3( [O65(1), 0], [O65(2), 0], [O65(3), 0], 'Parent', t5, 'linestyle', '--' )
    
    else
      t1 = handles(2);
      t2 = handles(3);
      t3 = handles(4);
      t4 = handles(5);
      t5 = handles(6);
      t6 = handles(7);
      
      set( t1, 'Matrix', T01 );
      set( t2, 'Matrix', T12 );
      set( t3, 'Matrix', T23 ),
      set( t4, 'Matrix', T34 );
      set( t5, 'Matrix', T45 );
      set( t6, 'Matrix', T56 );
      drawnow;
    
    end
    
    handlesR(1) = axs;
    handlesR(2) = t1;
    handlesR(3) = t2;
    handlesR(4) = t3;
    handlesR(5) = t4;
    handlesR(6) = t5;
    handlesR(7) = t6;

end
