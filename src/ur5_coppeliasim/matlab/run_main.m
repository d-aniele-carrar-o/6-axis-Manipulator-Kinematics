function run_main()
    parameters(2);

    disp('Program started');
    
    vrep = remApi( 'remoteApi' );
    vrep.simxFinish( -1 );
    id   = vrep.simxStart( '127.0.0.1', 19997, true, true, 2000, 5 );
    
    if id < 0
        disp('Failed connecting to remote API server. Exiting.');
        vrep.delete();
        return;
    end
    fprintf('Connection %d to remote API server open.\n', id);
    
    clear functions; % @callback_fun;
    
    % Make sure we close the connection whenever the script is interrupted.
    cleanupObj = onCleanup( @() cleanup_vrep( vrep, id ) );
    
    % This will only work in "continuous remote API server service"
    % See http://www.v-rep.eu/helpFiles/en/remoteApiServerSide.htm
    res = vrep.simxStartSimulation( id, vrep.simx_opmode_oneshot_wait );
    
    % Retrieve all handles and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    h = ur5_init( vrep, id );
    
    % Let a few cycles pass to make sure there's a value waiting for us next time
    % we try to get a joint angle or the robot pose with the simx_opmode_buffer option.
    pause(.2);
    
    q0 = [pi/2, -pi/6, 2*pi/3, 0, -pi/2, 0]
    T0 = direct_kinematics_cpp( q0, AL, A, D, TH )
    
    % Set the arm to its starting configuration:
    res = vrep.simxPauseCommunication( id, true ); vrchk( vrep, res );
    for i = 1:6
        res = vrep.simxSetJointTargetPosition( id, h.joints(i), q0(i), vrep.simx_opmode_oneshot ); vrchk( vrep, res, true );
    end
    res = vrep.simxPauseCommunication( id, false ); vrchk( vrep, res );
    
    % Make sure everything is settled before we start
    pause(2);

    % Create desired pose
    phi_f = [0, pi, 0];
    Rf    = eul2rotm_custom( phi_f );
    pf    = [-0.6; 0; 0.06];
    Tf    = [Rf, pf; 0,0,0,1]
    
    ti    = 0;

    viapoints = [Tf];
    times     = [ti, 1];

    % Compute multi-viapoint trajectory for selected times and viapoints
    % [t, p, v] = multipoint_trajectory( q0, viapoints, times );
    % N = size(p, 1);

    timestep = 0.01;
    t        = 0;
    j        = 1;

    [res, detection, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector] = vrep.simxReadProximitySensor( id, h.prox_sens, vrep.simx_opmode_streaming )
    
    while true
        tic

        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
        
        % Extract configuration from trajectory
        % q = p(j,:);
        
        % Update robot's joint configuration
        % res = vrep.simxPauseCommunication( id, true ); vrchk( vrep, res );
        % for i=1:6
        %     res = vrep.simxSetJointTargetPosition( id, h.joints(i), q(i), vrep.simx_opmode_oneshot ); vrchk( vrep, res, true );
        % end
        % res = vrep.simxPauseCommunication( id, false ); vrchk( vrep, res );
        
        [res, detection, detectedPoint, detectedObjectHandle, detectedSurfaceNormalVector] = vrep.simxReadProximitySensor( id, h.prox_sens, vrep.simx_opmode_buffer )
        
        if detection
            disp("oooooooooooooooooooooooooooooooooooooooooo")
        end

        % Make sure that we do not go faster that the simulator
        elapsed = toc;
        timeleft = timestep-elapsed;
        t = t + timestep;
        if (timeleft > 0)
            % pause(min(timeleft, .01));
            pause(timeleft)
        end
        
        % if j < N
        %     j = j+1;
        % end
        
    end

end

