function [T, v, angle] = get_velocity( qi, qf, T, tb )
    parameters(1)

    % compute velocity and check if max velocity is under max allowed
    if space == "joint"  %================================================%

        if traj_type == "LSPB"  %-----------------------------------------%
            % check if max velocity is under max allowed
            v = (qf-qi) / (T - tb);
            v_max = max( abs(v) );
            if v_max > max_vel
                v = v * max_vel / v_max;
                
                % check if max acceleration is under max allowed
                a = v / tb;
                a_max = max( abs(a) );
                if a_max > max_acc
                    a = a * max_acc / a_max;
                    v = a * tb;
                end
                T = max( abs(qf-qi) ) / max( abs(v) ) + tb;
            else
                % check if max acceleration is under max allowed
                a = v / tb;
                a_max = max( abs(a) );
                if a_max > max_acc
                    a = a * max_acc / a_max;
                    v = a * tb;
                    T = max( abs(qf-qi) ) / max( abs(v) ) + tb;
                end
            end
            angle = 0;
        
        elseif traj_type == "cubic"  %------------------------------------%
            % check if max velocity is under max allowed
            H     = (qf - qi);
            a2    =  3*H / T^2;
            a3    = -2*H / T^3;
            
            v     = 2*a2*T/2 + 3*a3*(T/2)^2;
            v_max = max( abs(v) );
            
            if v_max > max_vel
                T = T * v_max / max_vel;
                
                % check if max acceleration is under max allowed
                a     = 2*a2 + 6*a3*T;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = -a2 / (3*a3);
                end
            else
                % check if max acceleration is under max allowed
                a     = 2*a2 + 6*a3*T;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = sqrt( 6*max(abs(H)) / max_acc );
                end
            end
            angle = 0;
            
        elseif traj_type == "quintic"  %----------------------------------%
            % check if max velocity is under max allowed
            H    = (qf - qi);
            a3   =  10*H / T^3;
            a4   = -15*H / T^4;
            a5   =  10*H / T^5;
            
            v    = 3*a3*(T/2)^2 + 4*a4*(T/2)^3 + 5*a5*(T/2)^4;
            vmax = max( abs(v) );

            if vmax > max_vel
                T = 15/8 * max( abs(H) ) / max_vel;

                % check if max acceleration is under max allowed
                t0    = (3-sqrt(3))/6 * T;
                a     = 3*a3*t0 + 6*a4*t0^2 + 10*a5*t0^3;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = T * sqrt( a_max/max_acc );
                end
            else
                % check if max acceleration is under max allowed
                t0    = (3-sqrt(3))/6 * T;
                a     = 3*a3*t0 + 6*a4*t0^2 + 10*a5*t0^3;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = T * sqrt( a_max/max_acc );
                end
            end
            angle = 0;

        end

    elseif space == "task"  %=============================================%
        rotm_i_f   = qi(1:3,1:3)' * qf(1:3,1:3);
        [angle, ~] = rotm2angle_axis( rotm_i_f );
        q_f_i = [qf(1:3,4)' - qi(1:3,4)', angle];
        
        if traj_type == "LSPB"  %-----------------------------------------%
            % check if max velocity is under max allowed
            v = q_f_i / (T - tb);
            v_max = max( abs(v) );
            if v_max > max_vel
                v = v * max_vel / v_max;
                
                % check if max acceleration is under max allowed
                a = v / tb;
                a_max = max( abs(a) );
                if a_max > max_acc
                    a = a * max_acc / a_max;
                    v = a * tb;
                end
                T = max( abs(q_f_i) ) / max( abs(v) ) + tb;
            else
                % check if max acceleration is under max allowed
                a = v / tb;
                a_max = max( abs(a) );
                if a_max > max_acc
                    a = a * max_acc / a_max;
                    v = a * tb;
                    T = max( abs(q_f_i) ) / max( abs(v) ) + tb;
                end
            end

        elseif traj_type == "cubic"   %-----------------------------------%
            % check if max velocity is under max allowed
            H     = q_f_i;
            a2    =  3*H / T^2;
            a3    = -2*H / T^3;
            v     = 2*a2*T/2 + 3*a3*(T/2)^2;
            v_max = max( abs(v) );
            
            if v_max > max_vel
                T = T * v_max / max_vel;
                
                % check if max acceleration is under max allowed
                a     = 2*a2 + 6*a3*T;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = sqrt( 6*max( abs(H) ) / max_acc );
                end
            else
                % check if max acceleration is under max allowed
                a     = 2*a2 + 6*a3*T;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = sqrt( 6*max( abs(H) ) / max_acc );
                end
            end

        elseif traj_type == "quintic"  %----------------------------------%
            % check if max velocity is under max allowed
            H    = q_f_i;
            a3   =  10*H / T^3;
            a4   = -15*H / T^4;
            a5   =  10*H / T^5;
            
            v    = 3*a3*(T/2)^2 + 4*a4*(T/2)^3 + 5*a5*(T/2)^4;
            vmax = max( abs(v) );

            if vmax > max_vel
                T = 15/8 * max( abs(H) ) / max_vel;

                % check if max acceleration is under max allowed
                t0    = (3-sqrt(3))/6 * T;
                a     = 3*a3*t0 + 6*a4*t0^2 + 10*a5*t0^3;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = T * sqrt( a_max/max_acc );
                end
            else
                % check if max acceleration is under max allowed
                t0    = (3-sqrt(3))/6 * T;
                a     = 3*a3*t0 + 6*a4*t0^2 + 10*a5*t0^3;
                a_max = max( abs(a) );

                if a_max > max_acc
                    T = T * sqrt( a_max/max_acc );
                end
            end

        end
        
    end
end

