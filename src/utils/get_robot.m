function [robot, config] = get_robot( manipulator, Trf_0 )
    if manipulator == "UR5"
        robot = importrobot("robot_urdf/ur5.urdf");
    elseif manipulator == "UR3e"
        robot = importrobot( "robot_urdf/ur3e.urdf" );
        % Set the base transformation
        setFixedTransform(robot.Bodies{1}.Joint, Trf_0);
    elseif manipulator == "ABB"
        robot = importrobot("robot_urdf/ABB_IRb-7600.urdf");
    elseif manipulator == "custom"
        robot = importrobot("robot_urdf/custom_man.urdf");
    end
    config = robot.homeConfiguration;

end
