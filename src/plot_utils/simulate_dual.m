function [qf, axs] = simulate_dual(robots, configs, transforms, trajectories, axs)
    % SIMULATE_DUAL Simulates two robots with given trajectories
    %
    % Inputs:
    %   robots     - Cell array with {robot_left, robot_right}
    %   configs    - Cell array with {config_left, config_right}
    %   transforms - Cell array with {Trf_0_left, Trf_0_right}
    %   trajectories - Cell array with {p_left, p_right}
    %   axs        - Optional axes handle for plotting
    
    % Extract individual elements
    robot_left = robots{1};
    robot_right = robots{2};
    
    config_left = configs{1};
    config_right = configs{2};
    
    Trf_0_l = transforms{1};
    Trf_0_r = transforms{2};
    
    p_l = trajectories{1};
    p_r = trajectories{2};
    
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
    
    % Plot manipulator and scatter the positions of the end-effector to highlight trajectory 3D in space
    p_ee_l = Trf_0_l(1:3,4);
    p_ee_r = Trf_0_r(1:3,4);
    scatter3(axs, p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.'); hold on;
    scatter3(axs, p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.'); hold on;
    
    disp("Press button to start simulation..")
    k = waitforbuttonpress;
    while k ~= 1
        k = waitforbuttonpress;
    end
    
    disp("[simulate] Simulation started.")
    for i=2:max(size(p_l, 1), size(p_r, 1))
        if i <= size(p_l, 1)
            config_left = set_robot_configuration(p_l(i,:), config_left);
            show(robot_left, config_left, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs); hold on;
            
            % Get end-effector position
            parameters(1, 1); % Load parameters for robot 1
            [Te_w_e_left, ~] = direct_kinematics(p_l(i,1:6), 1);
            p_ee_l = Te_w_e_left(1:3,4);
            
            if (mod(i, 5) == 0)
                scatter3(axs, p_ee_l(1), p_ee_l(2), p_ee_l(3), 10, 'r.'); hold on;
            end
        end
        
        if i <= size(p_r, 1)
            config_right = set_robot_configuration(p_r(i,:), config_right);
            show(robot_right, config_right, "Visuals", "on", "Frames", "off", "FastUpdate", true, "PreservePlot", false, "Parent", axs); hold on;
            
            % Get end-effector position
            parameters(1, 2); % Load parameters for robot 2
            [Te_w_e_right, ~] = direct_kinematics(p_r(i,1:6), 2);
            p_ee_r = Te_w_e_right(1:3,4);
            
            if (mod(i, 5) == 0)
                scatter3(axs, p_ee_r(1), p_ee_r(2), p_ee_r(3), 10, 'b.'); hold on;
            end
        end
        
        waitfor(rate);
    end

    qf = {p_l(end,:), p_r(end,:)};

end
