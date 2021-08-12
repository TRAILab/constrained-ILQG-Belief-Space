classdef bearingSensor < ObservationModelBase
    %PLANAR_bearingSensor General bearing sensor

    
    properties(Constant = true)
        obsDim = 3;
        obsNoiseDim = 3;
        stDim = 3;
    end
    
    properties
        f_u = 484.4998;
        f_v = 484.4998;
        c_u = 321.6805;
        c_v = 247.4814;
        b = 0.24;
        var = [37.9799470231445;129.835565602725;41.9527461930872;132.489132838227];
        bear_var = 5.5068e-04;
        bear_var_noniso = [5.5068e-04;5.5068e-04;1e-6];
        C_cv = [0,-1,0;
                0,0,-1;
                1,0,0];
        psi = deg2rad(-15);
        FoV = deg2rad(60);
        max_alpha = deg2rad(165); %max parallax angle for feature matching
        eps = 1e-7;
        height = 0.5;
        
        k0 = 0.5;
        k1 = 0.5;
        k2 = 0;
        N_v = 10; %length of visibility approximation vector
        
        % FIF matrix
        FIF = [];
        ds = 0.1;
        
    end
    
    methods
        function obj = bearingSensor(landmarkIDs, landmarkPoses)
            % Construct an instance of this class
            %   Detailed explanation goes here
            obj@ObservationModelBase();
            obj.landmarkIDs = landmarkIDs;
            obj.landmarkPoses = landmarkPoses;
            
             % Duplication matrix: vec(P)=Dvech(P)
            n = obj.stDim;
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
        
        function [z,vis] = getObservation(obj, x, varargin)
            % getObservation, Get what sensor sees conditioned on varargin.            
            % getObservation(obj, x) all visible features with noise
            % getObservation(obj, x, 'nonoise') all visible features with no noise measurements
            % Output :
            % z Observ ation vector

                if nargin == 2 % noisy observations
                    noise = 1;  
                elseif nargin > 2 && strcmp('nonoise',varargin{1}) == 1 % nonoise
                    noise = 0;
                else                
                        error('unknown inputs')                
                end
                
                z = zeros(length(obj.landmarkIDs)*obj.obsDim,1);
                vis = zeros(length(obj.landmarkIDs)*obj.obsDim,1);
                for j = 1:length(obj.landmarkIDs)
                   [z_j, vis_j] = obj.getBearingMeas(obj.landmarkPoses(:,j), x);
                   if vis_j == 1
                        z(obj.obsDim*(j-1)+1: obj.obsDim*j) = z_j ...
                             + noise.*mvnrnd(zeros(obj.obsNoiseDim,1),diag(obj.bear_var).*eye(3),1).';
                        vis(obj.obsDim*(j-1)+1: obj.obsDim*j) = vis_j;                                       
                   end
                    
                end
                          
        end
        
        function [z,H] = getRealObservationAndJac(obj, x, varargin)
            % getObservation, Get what sensor sees conditioned on varargin.            
            % getObservation(obj, x) all visible features with noise
            % getObservation(obj, x, 'nonoise') all visible features with no noise measurements
            % Output :
            % z Observ ation vector
            % idfs Ids of observations
            [z_all, vis_all] = getObservation(obj, x);
            H_all = getObservationJacobian(obj, x,[]);
            z = z_all(vis_all>0);
            H = H_all(vis_all>0,:);
                
        end
        
        function [ y_j, isVisible ] = getBearingMeas(obj, p_ji_i, x_rob)
            %STEREO_PROJ Stereo projection model
            %inputs: p_ji_i - 3d point j in world frame, 4th dimension is
            %                 point normal direction
            %        x - robot state (x y theta)
            %outputs: y_j - stereo pixel coordinates, u_l,v_l,u_r,v_r
            
            
            th_j = p_ji_i(4);
            p_jc_c = obj.tf2camera_frame(p_ji_i(1:3),x_rob);
            y_j = p_jc_c./norm(p_jc_c);
            
            %Calculate visibility, FoV
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            %Calculate visibility, parallax angle alpha
            p_jk_i = -p_ji_i(1:3) + [x_rob(1:2);obj.height];
            p_jk_i_norm = p_jk_i./norm(p_jk_i);
            e_j = [cos(th_j);sin(th_j);0];
            
            alpha = acos(p_jk_i_norm.'*e_j);
            
            if theta <= obj.FoV/2 && alpha <=obj.max_alpha/2
                isVisible = 1;
            else
                isVisible = 0;
            end


        end
        
        function H = getObservationJacobian(obj, x,v)
            H = zeros(length(obj.landmarkIDs)*obj.obsDim,length(x));
            
            th = x(3); %robot heading
            p_vi_i = [x(1:2);obj.height]; % robot position in world frame,
            
            % Compute some necessary rotation matrices for Jacobian
            % computation
            
            C_th_i = bearingSensor.rot_C3(th);
            C_psi_th = bearingSensor.rot_C2(obj.psi);            
            C_psi_i = C_psi_th*C_th_i;
            C_ci = obj.C_cv * C_psi_i;
            
            for j = 1:length(obj.landmarkIDs)
               p_ji_i = obj.landmarkPoses(:,j);
               p_jc_c = obj.C_cv*C_psi_i*(p_ji_i(1:3) - p_vi_i);
               
               H_tf = bearingSensor.tfJac(C_ci, p_ji_i(1:3) - p_vi_i);
               H_proj = obj.bearingJac(p_jc_c);
               
               H(obj.obsDim*(j-1)+1:obj.obsDim*j,:) = H_proj * H_tf;
            end
        end
        
        function [ H_j ] = bearingJac(obj, p_jc_c)
            %BEARINGJAC Jacobian of bearing measurement wrt point in camera frame,
            %          needs 3 x 1 input point
            n_j = norm(p_jc_c);
            H_j = eye(3)./n_j - (1/(n_j^3)).*(p_jc_c*p_jc_c.');
            
        end
        
        function I_j = compute_FIM_actual(obj, p_jc_i)
           %Compute observed Fisher Information Matrix ignoring visibility
           
            n_j = norm(p_jc_i);
            I_xx = eye(3)./(n_j^2) - (1/(n_j^4)).*(p_jc_i*p_jc_i.');
            I_xtheta = -(1/(n_j^2)).*hat(p_jc_i);
            I_thetatheta = eye(3) - (1/(n_j^2)).* (p_jc_i*p_jc_i.');
            I_j = (1/obj.bear_var).*[I_xx, I_xtheta;
                    I_xtheta.', I_thetatheta];
            indices = [1,2,6];
            I_j = I_j(indices,indices);
            
        end        
        
        function C_I_j = vis_pos(obj,p_jc_i)
            % Compute the FIM positional factor for feature j, including visibility approx
            p_jc_i_norm = p_jc_i./norm(p_jc_i);
            p1 = p_jc_i_norm(1);
            p2 = p_jc_i_norm(2);
            p3 = p_jc_i_norm(3);
            
            v_p = [p1^2, p2^2, p3^2, p1*p2, p1*p3, p2*p3,p1,p2,p3,1].';
            
            I_j_actual = compute_FIM_actual(obj, p_jc_i);
            C_I_j = blkdiag(v_p,v_p,v_p)*I_j_actual;
        end
        
        function C_I = compute_FIM_pos(obj,x)
            % Compute FIM positional factor for a robot position
            p_ci_i = [x(1:2);obj.height];
            
            C_I = zeros(3*obj.N_v,3); 
            for j = 1:length(obj.landmarkIDs)
            % Calculate visibility, parallax angle alpha
                p_ji_i = obj.landmarkPoses(:,j);
                th_j = deg2rad(p_ji_i(4));
                
                p_jc_i = p_ci_i - p_ji_i(1:3);
                p_jc_i_norm = p_jc_i./norm(p_jc_i);
                e_j = [cos(th_j);sin(th_j);0];
                alpha = acos(p_jc_i_norm.'*e_j);
                
                if alpha > obj.max_alpha/2
                    continue
                else
                    C_I = C_I + vis_pos(obj,p_jc_i);
                end
                
            end
            
        end
        
        function V_I = compute_FIM_rot(obj,x)
            theta = x(3);
            C_th_i = bearingSensor.rot_C3(theta);

            C_psi_th = bearingSensor.rot_C2(obj.psi);

            C_ci = obj.C_cv*C_psi_th*C_th_i;
            z = C_ci.'*[0;0;1];
            
            z1 = z(1);
            z2 = z(2);
            z3 = z(3);
            
            v_r = [obj.k2*z1^2, obj.k2*z2^2, obj.k2*z3^2, ...
                2*obj.k2*z1*z2, 2*obj.k2*z1*z3, 2*obj.k2*z2*z3,...
                        obj.k1*z1,obj.k1*z2,obj.k1*z3,obj.k0];
            
            V_I = blkdiag(v_r,v_r,v_r);
        end
        
        function p_jc_c = tf2camera_frame(obj,p_ji_i, x)
            theta = x(3);
            
            p_jc_i = p_ji_i - [x(1:2);obj.height];
            C_th_i = bearingSensor.rot_C3(theta);

            C_psi_th = bearingSensor.rot_C2(obj.psi);

            C_ci = obj.C_cv*C_psi_th*C_th_i;
            
            p_jc_c = C_ci*p_jc_i;
            
        end
        
        function p_vis = visibility_probability(obj,p_jc_c,p_ji_i, x)
            %Calculate visibility, FoV angle theta
            
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            %Calculate visibility, parallax angle alpha
            th_j = deg2rad(p_ji_i(4));
            r_jc_i = -p_ji_i(1:3) + [x(1:2);obj.height];
            r_jc_i_norm = r_jc_i./norm(r_jc_i);
            e_j = [cos(th_j);sin(th_j);0];
            
            alpha = acos(r_jc_i_norm.'*e_j);
            
            if theta <= obj.FoV/2 && alpha <=obj.max_alpha/2
                p_vis = 0.5*(cos(pi*theta/(obj.FoV/2)) + 1)*0.5*(cos(pi*alpha/(obj.max_alpha/2)) + 1);
%                 p_vis = 0.5*(cos(pi*theta/(obj.FoV/2)) + 1);
            else
                p_vis = 0;
            end

        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            M = eye(length(obj.landmarkIDs)*obj.obsDim);
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
            
%            variances_all_j = repmat(obj.var,length(obj.landmarkIDs),1); 
           variances_all_j = repmat(obj.bear_var,3*length(obj.landmarkIDs),1);
           R = diag(variances_all_j);
          %Scale by visibility
           for j = 1:length(obj.landmarkIDs)
                p_ji_i = obj.landmarkPoses(:,j);
                p_jc_c = tf2camera_frame(obj,p_ji_i(1:3), x);
                p_vis_j = obj.visibility_probability(p_jc_c,p_ji_i,x);
                R(obj.obsDim*(j-1)+1: obj.obsDim*j,obj.obsDim*(j-1)+1: obj.obsDim*j) =...
                        (1/(p_vis_j+obj.eps)).*R(obj.obsDim*(j-1)+1: obj.obsDim*j,obj.obsDim*(j-1)+1: obj.obsDim*j);
                
           end
            
        end
        
        function R = getObservationNoiseCovarianceReal(obj,x,z)
            
%            variances_all_j = repmat(obj.var,length(obj.landmarkIDs),1);
           variances_all_j = repmat(obj.bear_var,3*length(obj.landmarkIDs),1);
           R = diag(variances_all_j);
            
        end
        
        function R_inv = getObservationNoiseCovarianceInverse(obj,x,z)
           variances_all_j = repmat(1./obj.bear_var,3*length(obj.landmarkIDs),1);
           R_inv = diag(variances_all_j);
           for j = 1:length(obj.landmarkIDs)
                p_ji_i = obj.landmarkPoses(:,j);
                p_jc_c = tf2camera_frame(obj,p_ji_i(1:3), x);
                p_vis_j = obj.visibility_probability(p_jc_c,p_ji_i,x);
                R_inv(obj.obsDim*(j-1)+1: obj.obsDim*j,obj.obsDim*(j-1)+1: obj.obsDim*j) =...
                        p_vis_j.*R_inv(obj.obsDim*(j-1)+1: obj.obsDim*j,obj.obsDim*(j-1)+1: obj.obsDim*j);
           end
        end
        
        function innov = computeInnovation(obj,Xprd,Zg)
            innov = Zg - obj.getObservation(Xprd, 'nonoise');
        end
        
        function set_vis_coeffs(obj, v_FoV)
            vis_0 = [1,1,1];
            val_0 = 1;
            
            vis_behind = [1,-1,1];
            val_behind = 0;
            
            vis_FoV = [1,cos(obj.FoV),cos(obj.FoV)^2];
            val_FoV = v_FoV;
            
            A = [vis_0;vis_behind;vis_FoV];
            B = [val_0;val_behind;val_FoV];
            
            coeffs = A\B;
            obj.k0 = coeffs(1);
            obj.k1 = coeffs(2);
            obj.k2 = coeffs(3);
            
        end
        
        function loadFIF(obj, FIF_filename)
           if ~isempty(obj.FIF)
               fprintf('FIF already loaded.\n')
           else  
               load(FIF_filename,'FIF');
               obj.FIF = FIF;
               fprintf('FIF loaded.\n')
           end
        end
        
        function FIF_pos = nearestFIF_pos(obj, x)
            x_nearest = round(x(1)/obj.ds) + 1;
            y_nearest = round(x(2)/obj.ds) + 1;
            

            FIF_pos = obj.FIF(x_nearest,y_nearest,:);

            FIF_pos = reshape(FIF_pos,obj.stDim*obj.N_v,obj.stDim);
        end
        

        
    end
    
    methods(Static)
        function p_3D = to3D(p_2D)
            %Convert 2D point in x y to 3D
            p_3D = [p_2D;0];
        end
        
        function p_4D = to4D(p_2D)
        %Convert 2D point in x y to a homogenous 3D point 
            p_4D = [p_2D;0;1];
        end
        
        function p_2D = to2D(p_XD)
        %Convert 3D or Homogenous 3D point to 2D point in x y
            p_2D = [p_XD(1);p_XD(2)];
        end
        
        function rotation_mat = rot_C3(theta)
            rotation_mat = [cos(theta), sin(theta), 0;
                            -sin(theta), cos(theta), 0;
                            0, 0, 1];
        end
        
        function rotation_mat = rot_C2(psi)
           rotation_mat = [cos(psi), 0, -sin(psi);
                            0, 1, 0;
                            sin(psi),0,cos(psi)];
        end
        
        function Z = tfJac(C_ci, p_jc_i)
            Z = zeros(3,3);
            
            Z(:,1:2) = -C_ci(:,1:2); % transformation jacobian wrt to position states
            Z(:,3) = C_ci*hat(p_jc_i)*[0;0;1]; % transformation jacobian wrt to heading

        end

    end
end


