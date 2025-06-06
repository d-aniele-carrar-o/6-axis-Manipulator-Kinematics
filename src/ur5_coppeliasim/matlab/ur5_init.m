function handles = ur5_init( vrep, id )
    handles = struct('id', id);

    joints  = zeros(6,1);

    [res, joints(1)] = vrep.simxGetObjectHandle( id, 'UR5_joint1', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, joints(2)] = vrep.simxGetObjectHandle( id, 'UR5_joint2', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, joints(3)] = vrep.simxGetObjectHandle( id, 'UR5_joint3', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, joints(4)] = vrep.simxGetObjectHandle( id, 'UR5_joint4', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, joints(5)] = vrep.simxGetObjectHandle( id, 'UR5_joint5', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, joints(6)] = vrep.simxGetObjectHandle( id, 'UR5_joint6', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    
    [res, base] = vrep.simxGetObjectHandle( id, 'UR5', vrep.simx_opmode_oneshot_wait); vrchk( vrep, res );
    [res, prox] = vrep.simxGetObjectHandle( id, 'Proximity_sensor', vrep.simx_opmode_oneshot_wait ); vrchk( vrep, res );

    handles.joints    = joints;
    handles.base      = base;
    handles.prox_sens = prox;

end

