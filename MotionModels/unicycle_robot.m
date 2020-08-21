classdef unicycle_robot < MotionModelBase
    %UNICYCLE_ROBOT Class definition for a unicycle robot
    %   Detailed explanation goes here
    
     properties (Constant = true) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim = 3; % state dimension
        ctDim = 2;  % control vector dimension
        wDim = 2;   % Process noise (W) dimension
        P_Wg = diag([0.15,0.1].^2); % covariance of state-additive-noise
        sigma_b_u = 0; % A constant bias intensity (covariance) of the control noise
        eta_u = 0; % A coefficient, which makes the control noise intensity proportional to the control signal       
        zeroNoise = [0;0];
        ctrlLim = [-5.0 5.0;-90*pi/180 90*pi/180]; % control limits
            turn_radius_min = 1.5*0.1; % indeed we need to define the minimum linear velocity in turnings (on orbits) and then find the minimum radius accordingly. But, we picked the more intuitive way.
        angular_velocity_max = 90*pi/180; % degree per second (converted to radian per second)
        linear_velocity_max = 0.5*10;
        linear_velocity_min_on_orbit = unicycle_robot.turn_radius_min*unicycle_robot.angular_velocity_max; % note that on the straight line the minimum velocity can go to zero. But, in turnings (on orbit) the linear velocity cannot fall below this value.
        D = [1,0,0,0,0,0;
             0,1,0,0,0,0;
             0,0,1,0,0,0;
             0,1,0,0,0,0;
             0,0,0,1,0,0;
             0,0,0,0,1,0;
             0,0,1,0,0,0;
             0,0,0,0,1,0;
             0,0,0,0,0,1;]; % Duplication matrix for creating full vectorized covariance matrix from diagnoal and below elements
         D_psuedoinv = [1,0,0,0,0,0,0,0,0;
                        0,0.5,0,0.5,0,0,0,0,0;
                        0,0,0.5,0,0,0,0.5,0,0;
                        0,0,0,0,1,0,0,0,0;
                        0,0,0,0,0,0.5,0,0.5,0;
                        0,0,0,0,0,0,0,0,1;];
     end
    

    methods
        function obj = unicycle_robot(dt)
            obj@MotionModelBase();      
            obj.dt = dt;
        end
        
        
        function x_next = evolve(obj,x,u,w) % discrete motion model equation
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
            A = [1, 0, -d_t*(u(1)+w(1))*s;
                  0, 1,  d_t*(u(1)+w(1))*c;
                  0, 0, 1];
        end
        
        function B = getControlJacobian(obj,x,u,w) % control Jacobian
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            B = d_t.*[c, 0;
                       s, 0;
                       0, 1];
            
        end
        
        function G = getProcessNoiseJacobian(obj,x,u,w) % noise Jacobian
            c = cos(x(3));
            s = sin(x(3));
            d_t = obj.dt;
            G = d_t.*[c, 0;
                       s, 0;
                       0, 1];
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
        
        function nominal_traj = generate_open_loop_point2point_traj(obj,x_initial,x_final,T)
            % "x_initial" and "x_final" are vectors that indicate the start
            % and final position of the state trajectory, we are planning
            % the control "up" for.
            % T is the length of the horizon
            % The nominal trajectory is simply rolling out with steady
            % state LQR controller
            n = obj.stDim;
            m = obj.ctDim;
            
            A = zeros(n,n);
            B = eye(n,m);
            Q = eye(n,n);
            R = 0.1.*eye(m,m);
            % R(3,3) = 3;
            
            A_k = obj.getStateTransitionJacobian(x_final,0,0);
            B_k = obj.getControlJacobian(x_final,0,0);

            K = lqr(A_k,B_k,Q,R);

            x_e_k = x_initial - x_final;
            nominal_traj.x = zeros(n,T+1);
            nominal_traj.u = zeros(m,T);
            
            nominal_traj.x(:,1) = x_initial;
            for k = 1:T
                u_k = -K*x_e_k;
                x_e_k = (A_k - B_k*K)*x_e_k;
                
                nominal_traj.x(:,k+1) = x_e_k + x_final;
                nominal_traj.u(:,k) = u_k;
            end

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

