function run_simulation( callback_fun, plotData, varargin )
    % (C) Copyright Renaud Detry 2013. Distributed under the GNU General Public License.
    % (See http://www.gnu.org/copyleft/gpl.html)
    
    disp('Program started');
    % Use the following line if you had to recompile remoteApi
    % vrep = remApi('remoteApi', 'extApi.h');
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
    
    % We're not checking the error code - if vrep is not run in continuous remote mode, 
    % simxStartSimulation could return an error.
    vrchk(vrep, res);
    
    % Retrieve all handles and stream arm and wheel joints, the robot's pose, the Hokuyo, and the arm tip pose.
    h = ur5_init( vrep, id );
    % h = youbot_hokuyo_init( vrep, h );
    
    
    % Let a few cycles pass to make sure there's a value waiting for us next time
    % we try to get a joint angle or the robot pose with the simx_opmode_buffer option.
    pause(.2);
    
    % Constants:
    timestep    = 0.05;
    wheelradius = 0.0937/2; % This value may be inaccurate. Check before using.
    
    % Min max angles for all joints:
    armJointRanges = [-2.9496064186096, 2.9496064186096;
                      -1.5707963705063, 1.3089967966080;
                      -2.2863812446594, 2.2863812446594;
                      -1.7802357673645, 1.7802357673645;
                      -1.5707963705063, 1.57079637050630 ];
    
    startingJoints = [0, 30.91*pi/180, 52.42*pi/180, 72.68*pi/180, 0];
    
    
    disp('Starting robot');
    
    % Set the arm to its starting configuration:
    res = vrep.simxPauseCommunication( id, true ); vrchk( vrep, res );
    
    for i = 1:5
        res = vrep.simxSetJointTargetPosition( id, h.armJoints(i), startingJoints(i), vrep.simx_opmode_oneshot ); vrchk( vrep, res, true );
    end

    res = vrep.simxPauseCommunication( id, false ); vrchk( vrep, res );
    
    % plotData = false;
    if plotData
        lidar_plot = figure;
        [X,Y] = meshgrid( -5:.25:5,-5.5:.25:2.5 );
        X     = reshape( X, 1, [] );
        Y     = reshape( Y, 1, [] );
        drawnow;
    end
    
    % Make sure everything is settled before we start
    pause(2);
    
    % [res, homeGripperPosition] = vrep.simxGetObjectPosition(id, h.ptip, h.armRef, vrep.simx_opmode_buffer);
    % vrchk(vrep, res, true);
    % fsm = 'init';
    
    while true
        tic
        
        if vrep.simxGetConnectionId(id) == -1
            error('Lost connection to remote API.');
        end
        
        % Read robot's position and orientation
        [res, youbotPos]   = vrep.simxGetObjectPosition(    id, h.ref, -1, vrep.simx_opmode_buffer ); vrchk( vrep, res, true );
        [res, youbotEuler] = vrep.simxGetObjectOrientation( id, h.ref, -1, vrep.simx_opmode_buffer ); vrchk( vrep, res, true );
        
        % Read data from sensor
        [pts, contacts]    = youbot_hokuyo( vrep, h, vrep.simx_opmode_buffer );
        
        if plotData
            figure(lidar_plot)
            cla;
            % Read data from the Hokuyo sensor:
            in = inpolygon( X, Y, [h.hokuyo1Pos(1) pts(1,:) h.hokuyo2Pos(1)], [h.hokuyo1Pos(2) pts(2,:) h.hokuyo2Pos(2)] );
            % subplot(211)
            plot(X(in), Y(in),'.g', pts(1,contacts), pts(2,contacts), '*r', [h.hokuyo1Pos(1), pts(1,:), h.hokuyo2Pos(1)], ...
            [h.hokuyo1Pos(2), pts(2,:), h.hokuyo2Pos(2)], 'r', 0, 0, 'ob', h.hokuyo1Pos(1), h.hokuyo1Pos(2), 'or', h.hokuyo2Pos(1), h.hokuyo2Pos(2), 'or');
            axis([-5.5 5.5 -5.5 0]);
            axis equal;
            drawnow;
        end
        
        [forwBackVel, leftRightVel, rotVel, finish] = callback_fun( pts, contacts, youbotPos, youbotEuler, varargin{:} );
        
        if finish
            pause(0.5);
            break;
        end
        
        % Update wheel velocities
        res = vrep.simxPauseCommunication( id, true ); vrchk( vrep, res );
        
        vrep.simxSetJointTargetVelocity( id, h.wheelJoints(1), -forwBackVel-leftRightVel+rotVel, vrep.simx_opmode_oneshot ); vrchk( vrep, res );
        vrep.simxSetJointTargetVelocity( id, h.wheelJoints(2), -forwBackVel+leftRightVel+rotVel, vrep.simx_opmode_oneshot ); vrchk( vrep, res );
        vrep.simxSetJointTargetVelocity( id, h.wheelJoints(3), -forwBackVel-leftRightVel-rotVel, vrep.simx_opmode_oneshot ); vrchk( vrep, res );
        vrep.simxSetJointTargetVelocity( id, h.wheelJoints(4), -forwBackVel+leftRightVel-rotVel, vrep.simx_opmode_oneshot ); vrchk( vrep, res );
        
        res = vrep.simxPauseCommunication( id, false ); vrchk( vrep, res );
        
        % Make sure that we do not go faster that the simulator
        elapsed = toc;
        % disp(elapsed);
        timeleft = timestep-elapsed;
        if (timeleft > 0)
            pause(min(timeleft, .01));
        end
        
    end

end % main function

