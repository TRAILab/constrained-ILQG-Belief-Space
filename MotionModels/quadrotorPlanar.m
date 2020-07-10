classdef quadrotorPlanar < MotionModelBase
    %quadrotorPlanar Class definition for a unicycle robot
    %   Simplified single integrator model for quadrotor in the plane
    %   States are x,y,theta (-pi<theta<pi)
    %   Control inputs are v_x, v_y, theta_dot
    
     properties (Constant = true) % note that all properties must be constant, because we have a lot of copies of this object and it can take a lot of memory otherwise.
        stDim = 3; % state dimension
        ctDim = 3;  % control vector dimension
        wDim = 3;   % Process noise (W) dimension
        P_Wg = diag([0.15,0.15,0.09].^2); % covariance of state-additive-noise
        sigma_b_u = 0; % A constant bias intensity (covariance) of the control noise
        eta_u = 0; % A coefficient, which makes the control noise intensity proportional to the control signal       
        zeroNoise = [0;0;0];
        ctrlLim = [-5.0 5.0;-5 5; -90*pi/180 90*pi/180]; % control limits
     end
    

    methods
        function obj = quadrotorPlanar(dt)
            obj@MotionModelBase();      
            obj.dt = dt;
        end
        
        
        function x_next = evolve(obj,x,u,w) % discrete motion model equation
            d_t = obj.dt;
            x_next = x + d_t.*(u+w);
            
        end
        
        function A = getStateTransitionJacobian(obj,x,u,w) % state Jacobian
            A = eye(3);
        end
        
        function B = getControlJacobian(obj,x,u,w) % control Jacobian
            B = obj.dt.*eye(3);           
            
        end
        
        function G = getProcessNoiseJacobian(obj,x,u,w) % noise Jacobian
            G = obj.dt.*eye(3);
            %G = obj.dt.*eye(3);

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
            B = eye(m,m);
            Q = eye(n,n);
            R = 0.1.*eye(m,m);
            R(3,3) = 3;
            K = lqr(A,B,Q,R);
            
            A_k = obj.getStateTransitionJacobian(0,0,0);
            B_k = obj.getControlJacobian(0,0,0);

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
        

    end
    

end

