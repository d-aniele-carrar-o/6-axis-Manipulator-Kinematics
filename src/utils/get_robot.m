function [robot] = get_robot(dhparams, Trf_0, real_robot)
    if real_robot
        % Load the Universal Robots UR3e model
        robot = loadrobot("universalUR3e");
        setFixedTransform(robot.Bodies{2}.Joint, Trf_0);
    else
        % Create a custom robot using DH parameters
        bodies = cell(7,1);
        joints = cell(7,1);
        
        % Initialize the rigid body tree
        robot = rigidBodyTree;
        
        % Create a fixed base
        bodies{1} = rigidBody('base_link');
        joints{1} = rigidBodyJoint('base_joint', 'fixed');
        
        % Set the base transformation
        setFixedTransform(joints{1}, Trf_0);
        bodies{1}.Joint = joints{1};
        
        % Add the base to the robot
        addBody(robot, bodies{1}, 'base');
        
        % Create a simple 6-DOF robot with visual elements
        for i=2:7
            % Create body and joint
            bodies{i} = rigidBody(['body' num2str(i-1)]);
            joints{i} = rigidBodyJoint(['jnt' num2str(i-1)], "revolute");
            
            % Set joint properties based on which joint it is
            setFixedTransform(joints{i}, dhparams(i-1,:), "dh");
            bodies{i}.Joint = joints{i};
            
            % Add body to the tree - first link connects to base_link, others to previous body
            if i == 2
                addBody(robot, bodies{i}, 'base_link');
            else
                addBody(robot, bodies{i}, ['body' num2str(i-2)]);
            end
        end
    end

    showdetails(robot)
    
end
