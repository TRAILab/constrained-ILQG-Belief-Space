classdef car_panTilt < MotionModelBase
    %UNICYCLE_ROBOT Class definition for a car robot with a pan-tilt camera
    %   state x is assumed to be [x_pos,y_pos,theta,phi,psi]
    %   theta - heading of robot on plane
    %   phi - pan angle
    %   psi - tilt angle
    
    properties (Constant = true) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim = 5; % state dimension
        ctDim = 4;  % control vector dimension
        wDim = 4;   % Process noise (W) dimension
        P_Wg = diag(([0.4,0.07,0.05,0.05]./3).^2); % covariance of state-additive-noise
        sigma_b_u = 0; % A constant bias intensity (covariance) of the control noise
        eta_u = 0; % A coefficient, which makes the control noise intensity proportional to the control signal       
        zeroNoise = [0;0;0;0];
        ctrlLim = [-5.0 5.0;-90*pi/180 90*pi/180]; % control limits
        turn_radius_min = 1.5*0.1; % indeed we need to define the minimum linear velocity in turnings (on orbits) and then find the minimum radius accordingly. But, we picked the more intuitive way.
        angular_velocity_max = 90*pi/180; % degree per second (converted to radian per second)
        linear_velocity_max = 0.5*10;
        linear_velocity_min_on_orbit = unicycle_robot.turn_radius_min*unicycle_robot.angular_velocity_max; % note that on the straight line the minimum velocity can go to zero. But, in turnings (on orbit) the linear velocity cannot fall below this value.
       % D = []; % Duplication matrix for creating full vectorized covariance matrix from diagnoal and below elements
%         D_psuedoinv = [1,0,0,0,0,0,0,0,0;
%                         0,0.5,0,0.5,0,0,0,0,0;
%                         0,0,0.5,0,0,0,0.5,0,0;
%                         0,0,0,0,1,0,0,0,0;
%                         0,0,0,0,0,0.5,0,0.5,0;
%                         0,0,0,0,0,0,0,0,1;];
    end
    

    methods
        function obj = car_panTilt(dt)
            obj@MotionModelBase();      
            obj.dt = dt;
            n=5;
            % Duplication matrix: vec(P)=Dvech(P)
            m=1/2*n*(n+1);
            nsq=n^2;
            DT=sparse(m,nsq);
            for j=1:n
                for i=j:n
                    ijth=(j-1)*n+i;
                    jith=(i-1)*n+j;
                    vecTij=sparse(ijth,1,1,nsq,1);
                    vecTij(jith,1)=1;
                    k=(j-1)*n+i-1/2*j*(j-1);
                    uij=sparse(k,1,1,m,1);
                    DT=DT+uij*vecTij';
                end
            end
            D=DT';
            obj.D = full(D);
            obj.D_pseudoinv = (D'*D)\(D');

        end
        
        
        function x_next = evolve(obj,x,u,w) % discrete motion model equation
%             Un = w(1:unicycle_robot.ctDim); % The size of Un may be different from ctDim in some other model.
%             Wg = w(Unicycle_robot.ctDim+1 : Unicycle_robot.wDim); % The size of Wg may be different from stDim in some other model.
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            x_next = x + d_t.*[c,0,0,0;
                               s,0,0,0;
                               0,1,0,0;
                               0,0,1,0;
                               0,0,0,1]*(u+w);
            
        end
        
        function x_next = evolve_unicycle(obj,x,u,w) % discrete motion model equation
%             Un = w(1:unicycle_robot.ctDim); % The size of Un may be different from ctDim in some other model.
%             Wg = w(Unicycle_robot.ctDim+1 : Unicycle_robot.wDim); % The size of Wg may be different from stDim in some other model.
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            x_next = x + d_t.*[c,0;s,0;0,1]*(u+w);
            
        end
        
        function A = getStateTransitionJacobian(obj,x,u,w) % state Jacobian
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            A = [1, 0, -d_t*(u(1)+w(1))*s,0,0;
                  0, 1,  d_t*(u(1)+w(1))*c,0,0;
                  0, 0, 1, 0,0 ;
                  0, 0, 0, 1, 0;
                  0, 0, 0, 0, 1];
        end
        
        function B = getControlJacobian(obj,x,u,w) % control Jacobian
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            B = d_t.*[c,0,0,0;
                      s,0,0,0;
                      0,1,0,0;
                      0,0,1,0;
                      0,0,0,1];
            
        end
        
        function G = getProcessNoiseJacobian(obj,x,u,w) % noise Jacobian
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            G = d_t.*[c+0.001,0,0,0;
                      s+0.001,0,0,0;
                      0,1.000001,0,0;
                      0,0,1,0;
                      0,0,0,1];
%             G = [c, 0;
%            s, 0;
%            0, 1];
        end
        
        function Q = getProcessNoiseCovariance(obj,x,u)
            Q = obj.P_Wg;
        end
        
        function w = generateProcessNoise(obj,x,u)
           w = mvnrnd(zeros(obj.wDim,1),obj.P_Wg)'; 
        end
        
        function U = generateOpenLoopControls(obj,x0,xf)   
           nominal_traj = obj.generate_open_loop_point2point_traj(x0,xf);
           U = nominal_traj.u;
        end
        
        function nominal_traj = generate_open_loop_point2point_traj(obj,x_initial,x_final)
            % "x_initial" and "x_final" are vectors that indicate the start
            % and final position of the state trajectory, we are planning
            % the control "up" for.
            
            % minimum turn radius resutls from dividing the minimum linear
            % velocity to maximum angular velocity. However, here we assume
            % that the linear velocity is constant.
            radius = unicycle_robot.turn_radius_min;
            line_rough_tangent = atan2(x_final(2) - x_initial(2), x_final(1) - x_initial(1)); % angle of line connecting two waypoints
            
            initial_heading_change = mod(line_rough_tangent - x_initial(3), 2*pi);
            if initial_heading_change <= pi % heading turn ccw
                initial_heading_turn = -1;
                initial_turn_str = 'ccw';
            else % heading turn cw
                initial_heading_turn = 1;
                initial_turn_str = 'cw';
            end
            initial_circle_center = [radius * cos(x_initial(3) - initial_heading_turn * pi/2) ; ...
                                     radius * sin(x_initial(3) - initial_heading_turn * pi/2)] ...
                                     + x_initial(1:2);
            final_heading_change = mod(x_final(3) - line_rough_tangent, 2*pi);
            if final_heading_change <= pi % heading turn ccw
                final_heading_turn = -1;
                final_turn_str = 'ccw';
            else % heading turn cw
                final_heading_turn = 1;
                final_turn_str = 'cw';
            end
            final_circle_center = [radius * cos(x_final(3) - final_heading_turn * pi/2) ; ...
                                   radius * sin(x_final(3) - final_heading_turn * pi/2)] ...
                                   + x_final(1:2);
            %             tth = 0:0.1:2*pi+.1;plot(initial_circle_center(1)+radius*cos(tth), initial_circle_center(2)+radius*sin(tth)); %TO DEBUG -  DONT DELETE
            %             tth = 0:0.1:2*pi+.1;plot(final_circle_center(1)+radius*cos(tth), final_circle_center(2)+radius*sin(tth)); %TO DEBUG -  DONT DELETE
            
            % gamma_tangent = atan2( final_circle_center(2) - initial_circle_center(2), final_circle_center(1) - initial_circle_center(1) ); % The angle of the tangent line % Deprecated
            gamma_start_of_tangent_line = line_rough_tangent + initial_heading_turn * pi/2; % the angle on which the starting point of the tangent line lies on orbit i.
            gamma_end_of_tangent_line = line_rough_tangent + final_heading_turn * pi/2; % the angle on which the ending point of the tangent line lies on orbit i.
            
            initial_robot_gamma = x_initial(3) + initial_heading_turn * pi/2; % Note that this is not robot's heading angle. This says that at which angle robot lies on the circle.
            final_robot_gamma = x_final(3) + final_heading_turn * pi/2; % Note that this is not robot's heading angle. This says that at which angle robot lies on the circle.
            
            % Turn part on the first circle
            entire_th_on_initial_circle = obj.delta_theta_turn(initial_robot_gamma, gamma_start_of_tangent_line, initial_turn_str);
            delta_theta_on_init_turns = -initial_heading_turn * unicycle_robot.angular_velocity_max * obj.dt;
            kf_pre_rational = entire_th_on_initial_circle / delta_theta_on_init_turns; 
            kf_pre = ceil(kf_pre_rational);
            V_pre = unicycle_robot.linear_velocity_min_on_orbit * [ones(1,kf_pre-1) , kf_pre_rational-floor(kf_pre_rational)];
            omega_pre = -initial_heading_turn * unicycle_robot.angular_velocity_max * [ones(1,kf_pre-1) , kf_pre_rational-floor(kf_pre_rational)];
            u_pre = [V_pre; omega_pre];
            w_zero = zeros(unicycle_robot.wDim,1); % no noise
            x_pre(:,1) = x_initial;
            for k=1:kf_pre
                x_pre(:,k+1) = obj.evolve_unicycle(x_pre(:,k),u_pre(:,k),w_zero);
                %                 tmp = state(x_pre(:,k+1));tmp.draw(); % FOR DEBUGGING
            end

            % Turn part on the final circle
            th_on_final_circle = obj.delta_theta_turn(gamma_end_of_tangent_line, final_robot_gamma, final_turn_str);
            delta_theta_on_final_turns = -final_heading_turn * unicycle_robot.angular_velocity_max * obj.dt;
            kf_post_rational = th_on_final_circle / delta_theta_on_final_turns;
            kf_post = ceil(kf_post_rational);
            V_post = unicycle_robot.linear_velocity_min_on_orbit * [ones(1,kf_post-1) , kf_post_rational-floor(kf_post_rational)];
            omega_post = -final_heading_turn * unicycle_robot.angular_velocity_max * [ones(1,kf_post-1) , kf_post_rational-floor(kf_post_rational)];
            u_post = [-V_post; -omega_post];
            x_post(:,kf_post+1) = x_final;
            for k=kf_post+1:-1:2 % calculate in reverse order, from x_final
                x_post(:,k-1) = obj.evolve_unicycle(x_post(:,k),u_post(:,k-1),w_zero);
                %                 tmp = state(x_post(:,k+1));tmp.draw(); % FOR DEBUGGING
            end

            % Line part, this is calculated last because the end points needs to be there first
            tanget_line_length = norm(x_post(:,1) - x_pre(:,kf_pre+1));
            step_length = unicycle_robot.linear_velocity_max * obj.dt;
            kf_line_rational = tanget_line_length / step_length;
            kf_line = ceil(kf_line_rational);
            V_line = unicycle_robot.linear_velocity_max * [ones(1,kf_line-1) , kf_line_rational-floor(kf_line_rational)];
            omega_line = zeros(1,kf_line);
            u_line = [V_line; omega_line];
            x_line(:,1) = x_pre(:,kf_pre+1);
            for k=1:kf_line
                x_line(:,k+1) = obj.evolve_unicycle(x_line(:,k),u_line(:,k),w_zero);
                %                 tmp = state(x_line(:,k+1));tmp.draw(); % FOR DEBUGGING
            end
            
            nominal_traj.x = [x_pre(:,1:kf_pre) , x_line(:,1:kf_line) , x_post(:,1:kf_post+1)]; % This line is written very carefully. So, dont worry about its correctness!
            nominal_traj.u = [u_pre(:,1:kf_pre) , u_line(:,1:kf_line) , -u_post(:,1:kf_post)]; % This line is written very carefully. So, dont worry about its correctness!
            
        end
        
        function delta_th = delta_theta_turn(obj,th_initial, th_final, direction)
        % this function returns the angle difference in the [-2pi, 2pi] range

            if strcmpi( direction, 'cw' ) % check if the direction is clockwise
                th_init_0_2pi = mod(th_initial, 2*pi) ; % here we shift the "th_initial"to the [0,2pi] range
                th_fin_0_2pi = mod(th_final, 2*pi) ; % here we shift the "th_final"to the [0,2pi] range
                if th_init_0_2pi >= th_fin_0_2pi % This means we do not pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi" in the cw direction.
                    delta_th = th_fin_0_2pi - th_init_0_2pi;
                else % This means we have to pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi". Therefore, in this case, we first go to zero from "th_init_0_2pi" and then from zero we go to the "th_fin_0_2pi".
                    th_fin_negative2pi_0 = th_fin_0_2pi - 2*pi; % here we shift the "th_final" to the [-2pi,0] range
                    delta_th = th_fin_negative2pi_0 - th_init_0_2pi;
                end
            elseif strcmpi( direction, 'ccw' ) % check if the direction is counterclockwise
                th_init_0_2pi = mod(th_initial, 2*pi) ; % here we shift the "th_initial"to the [0,2pi] range
                th_fin_0_2pi = mod(th_final, 2*pi) ; % here we shift the "th_final"to the [0,2pi] range
                if th_init_0_2pi <= th_fin_0_2pi % This means we do not pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi" in the ccw direction.
                    delta_th = th_fin_0_2pi - th_init_0_2pi;
                else % This means we have to pass the zero in  going from "th_init_0_2pi" to "th_fin_0_2pi". Therefore, in this case, we first go to zero from "th_init_0_2pi" and then from zero we go to the "th_fin_0_2pi".
                    th_init_negative2pi_0 = th_init_0_2pi - 2*pi; % here we shift the "th_init" to the [-2pi,0] range
                    delta_th = th_fin_0_2pi - th_init_negative2pi_0;
                end
            else
                error('The direction must be either "cw" (clockwise) or "ccw" (counterclockwise)')
            end

        end
    end
    
%     methods(Static)
%         function traj_plot_handle = draw_nominal_traj(nominal_traj, traj_flag)
%             traj_plot_handle = [];
%             if traj_flag == 1
%                 for k = 1 : size(nominal_traj.x , 2)
%                     tmp_Xstate = state (nominal_traj.x(:,k) );
%                     tmp_Xstate.draw('RobotShape','triangle','robotsize',1);%,'TriaColor',color(cycles));
%                     %traj_plot_handle(k:k+2) =
%                     %[tmp_Xstate.head_handle,tmp_Xstate.text_handle,tmp_Xstate.tria_handle];
%                 end
%             else
%                 tmp_handle = plot(nominal_traj.x(1,:) , nominal_traj.x(2,:));
%                 traj_plot_handle = [traj_plot_handle , tmp_handle];
%                 len = size( nominal_traj.x , 2);
%                 tmp_Xstate = state( nominal_traj.x(:,floor(len/2)) ); % to plot the direction of the line.
% %                 tmp_Xstate = tmp_Xstate.draw('RobotShape','triangle','robotsize',2);
% %                 traj_plot_handle = [traj_plot_handle , tmp_Xstate.plot_handle , tmp_Xstate.head_handle , tmp_Xstate.tria_handle , tmp_Xstate.text_handle ];
%                 drawnow
%             end
%         end
%        
%         
%     end
end

