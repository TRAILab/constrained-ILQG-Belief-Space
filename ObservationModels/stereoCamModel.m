classdef stereoCamModel < ObservationModelBase
    %PLANAR_STEREOCAMMODEL Camera model for a calibrated stereo camera
    %   state x is assumed to be [x_pos,y_pos,theta]
    %   theta - heading of robot on plane

    
    properties(Constant = true)
        obsDim = 4;
        obsNoiseDim = 4;
    end
    
    properties
        f_u = 484.4998;
        f_v = 484.4998;
        c_u = 321.6805;
        c_v = 247.4814;
        b = 0.24;
        var = [37.9799470231445;129.835565602725;41.9527461930872;132.489132838227];
        C_cv = [0,-1,0;
                0,0,-1;
                1,0,0];
        psi = deg2rad(-15);
        FoV = deg2rad(60);
        max_alpha = deg2rad(165); %max parallax angle for feature matching
        eps = 1e-7;
        height = 0.5;
    end
    
    methods
        function obj = stereoCamModel(landmarkIDs, landmarkPoses)
            %PLANAR_STEREOCAMMODEL Construct an instance of this class
            %   Detailed explanation goes here
            obj@ObservationModelBase();
            obj.landmarkIDs = landmarkIDs;
            obj.landmarkPoses = landmarkPoses;
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
                   [z_j, vis_j] = obj.stereo_proj(obj.landmarkPoses(:,j), x);
                   if vis_j == 1
                        z(obj.obsDim*(j-1)+1: obj.obsDim*j) = z_j ...
                             + noise.*mvnrnd(zeros(obj.obsNoiseDim,1),diag(obj.var),1).';
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
        
        function [ y_j, isVisible ] = stereo_proj(obj, p_ji_i, x_rob)
            %STEREO_PROJ Stereo projection model
            %inputs: p_ji_i - 3d point j in world frame, 4th dimension is
            %                 point normal direction
            %        x - robot state (x y theta)
            %outputs: y_j - stereo pixel coordinates, u_l,v_l,u_r,v_r
            
            p_jc_c = obj.tf2camera_frame(p_ji_i(1:3),x_rob);
            x = p_jc_c(1);
            y = p_jc_c(2);
            z = p_jc_c(3);
            th_j = p_ji_i(4);

            y_j = zeros(4,1);

            y_j(1) = obj.f_u*x/z + obj.c_u;
            y_j(2) = obj.f_v*y/z + obj.c_v;
            y_j(3) = obj.f_u*(x-obj.b)/z + obj.c_u;
            y_j(4) = obj.f_v*y/z + obj.c_v;
            
            %Calculate visibility, FoV
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            %Calculate visibility, parallax angle alpha
            r_jk_i = -p_ji_i(1:3) + [x_rob(1:2);obj.height];
            r_jk_i_norm = r_jk_i./norm(r_jk_i);
            e_j = [cos(th_j);sin(th_j);0];
            
            alpha = acos(r_jk_i_norm.'*e_j);
            
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
            
            C_th_i = stereoCamModel.rot_C3(th);
            C_psi_th = stereoCamModel.rot_C2(obj.psi);            
            C_psi_i = C_psi_th*C_th_i;
            
            for j = 1:length(obj.landmarkIDs)
               p_ji_i = obj.landmarkPoses(:,j);
               p_jc_c = obj.C_cv*C_psi_i*(p_ji_i(1:3) - p_vi_i);
               
               H_tf = stereoCamModel.tfJac(C_th_i,C_psi_i,p_ji_i(1:3) - p_vi_i);
               H_proj = obj.stereoJac(p_jc_c);
               
               H(obj.obsDim*(j-1)+1:obj.obsDim*j,:) = H_proj * obj.C_cv*H_tf;
            end
        end
        
        function [ H_j ] = stereoJac(obj, p_jc_c)
            %STEREOJAC Jacobian of stereo projection, needs 3 x 1 input point 

            x = p_jc_c(1);
            y = p_jc_c(2);
            z = p_jc_c(3);
            
            G_proj = [obj.f_u/z, 0, -obj.f_u*x/(z^2);
                      0, obj.f_v/z, -obj.f_v*y/(z^2);
                      obj.f_u/z,0,-obj.f_u*(x-obj.b)/(z^2);
                      0, obj.f_v/z, -obj.f_v*y/(z^2)]; 
            
            H_j = G_proj;
        end
        
        function p_jc_c = tf2camera_frame(obj,p_ji_i, x)
            theta = x(3);
            
            p_jc_i = p_ji_i - [x(1:2);obj.height];
            C_th_i = stereoCamModel.rot_C3(theta);

            C_psi_th = stereoCamModel.rot_C2(obj.psi);

            C_ci = obj.C_cv*C_psi_th*C_th_i;
            
            p_jc_c = C_ci*p_jc_i;
            
        end
        
        function p_vis = visibility_probability(obj,p_jc_c,p_ji_i, x)
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            %Calculate visibility, parallax angle alpha
            th_j = deg2rad(p_ji_i(4));
            r_jk_i = -p_ji_i(1:3) + [x(1:2);obj.height];
            r_jk_i_norm = r_jk_i./norm(r_jk_i);
            e_j = [cos(th_j);sin(th_j);0];
            
            alpha = acos(r_jk_i_norm.'*e_j);
            
            if theta <= obj.FoV/2 && alpha <=obj.max_alpha/2
                p_vis = 0.5*(cos(pi*theta/(obj.FoV/2)) + 1)*0.5*(cos(pi*alpha/(obj.max_alpha/2)) + 1);
            else
                p_vis = 0;
            end

        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            M = eye(length(obj.landmarkIDs)*obj.obsDim);
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
            
           variances_all_j = repmat(obj.var,length(obj.landmarkIDs),1);          
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
            
           variances_all_j = repmat(obj.var,length(obj.landmarkIDs),1);          
           R = diag(variances_all_j);
            
        end
        
        function R_inv = getObservationNoiseCovarianceInverse(obj,x,z)
           variances_all_j = repmat(1./obj.var,length(obj.landmarkIDs),1);
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
        
        function Z = tfJac( C_th_i, C_psi_i, p_jc_i)
            Z = zeros(3,3);
            
            Z(:,1:2) = -C_psi_i(:,1:2); % transformation jacobian wrt to position states
            Z(:,3) = C_psi_i*hat(p_jc_i)*(C_th_i.')*[0;0;1]; % transformation jacobian wrt to heading

        end
        

    end
end


