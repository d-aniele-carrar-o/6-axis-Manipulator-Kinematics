function [qf, axs] = simulate_dual(robots, configs, transforms, times, positions, velocities, axs)
    % SIMULATE_DUAL Simulates two robots with given trajectories
    %
    % Inputs:
    %   robots       - Cell array with {robot_left, robot_right}
    %   configs      - Cell array with {config_left, config_right}
    %   transforms   - Cell array with {Trf_0_left, Trf_0_right}
    %   trajectories - Cell array with {p_left, p_right}
    %   axs          - Optional axes handle for plotting
    
    parameters(1);

    % Extract individual elements
    robot_left = robots{1};
    robot_right = robots{2};
    
    config_left = configs{1};
    config_right = configs{2};
    
    Trf_0_l = transforms{1};
    Trf_0_r = transforms{2};

    t_l  = times{1};
    t_r  = times{2};
    
    ps_l = positions{1};
    ps_r = positions{2};

    vs_l = velocities{1};
    vs_r = velocities{2};

    qi_l = [config_left.JointPosition];
    qi_r = [config_right.JointPosition];
    
    % Get rate from caller workspace if available
    try
        rate = evalin('caller', 'rate');
    catch
        rate = rateControl(25); % Default rate if not provided
    end
    
    % Use provided axes or get from caller
    if nargin < 5
        try
            axs = evalin('caller', 'axs');
        catch
            figure('Name', 'Dual Robot Simulation');
            axs = gca;
            hold(axs, 'on'); grid(axs, 'on');
        end
    end

    % Transform task space positions to joint space for simulation
    if space == "task"
        % Transform the end-effector poses using IK of selected manipulator to joint space configurations
        [qs_l, qdots_l] = task2joint_space( qi_l, ps_l, vs_l );
        [qs_r, qdots_r] = task2joint_space( qi_r, ps_r, vs_r );
        
        if plot_grahps
            plot_joint_trajectory_pos_and_vel(t_l, ps_l, vs_l, "task");
            plot_joint_trajectory_pos_and_vel(t_l, qs_l, qdots_l, "joint");
            plot_joint_trajectory_pos_and_vel(t_r, ps_r, vs_r, "task");
            plot_joint_trajectory_pos_and_vel(t_r, qs_r, qdots_r, "joint");
        end

    elseif space == "joint"
        qs_l = ps_l;
        qs_r = ps_r;

    end
    
    % Plot manipulator and scatter the positions of the end-effector to highlight trajectory 3D in space
    p_ee_l = Trf_0_l(1:3,4);
    p_ee_r = Trf_0_r(1:3,4);
    scatter3(axs, p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.'); hold on;
    scatter3(axs, p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.'); hold on;
    
    disp("Press button to start simulation..")
    pause()
    
    disp("[simulate] Simulation started.")
    for i=2:max(size(qs_l, 1), size(qs_r, 1))
        if i <= size(qs_l, 1)
            config_left = set_robot_configuration(qs_l(i,:), config_left);
            show(robot_left, config_left, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs); hold on;
            
            % Get end-effector position, if not task space
            if space == "joint" || kinematics == "IDK"
                parameters(1, 1); % Load parameters for robot 1
                [Te_w_e_left, Te_l] = direct_kinematics(qs_l(i,1:6), 1);
            else
                Te_l = ps_l(4*(i-1)+1:4*i,:);
                Te_w_e_left = Trf_0_l * Te_l;
            end
            p_ee_l = Te_w_e_left(1:3,4);
            
            if (mod(i, 10) == 0)
                scatter3(axs, p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.'); hold on;
            end
        end
        
        if i <= size(qs_r, 1)
            config_right = set_robot_configuration(qs_r(i,:), config_right);
            show(robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs); hold on;
            
            % Get end-effector position, if not task space
            if space == "joint" || kinematics == "IDK"
                parameters(1, 1); % Load parameters for robot 1
                [Te_w_e_right, Te_r] = direct_kinematics(qs_r(i,1:6), 1);
            else
                Te_r = ps_r(4*(i-1)+1:4*i,:);
                Te_w_e_right = Trf_0_r * Te_r;
            end
            p_ee_r = Te_w_e_right(1:3,4);
            
            if (mod(i, 10) == 0)
                scatter3(axs, p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.'); hold on;
            end
        end
        
        waitfor(rate);

        % Break if figure is closed
        if ~ishandle(axs.Parent)
            break;
        end
    end

    if space == "joint"
        qf = {qs_l(end,:), qs_r(end,:)};
    elseif space == "task"
        qf = {Te_l, Te_r};
    end

end
