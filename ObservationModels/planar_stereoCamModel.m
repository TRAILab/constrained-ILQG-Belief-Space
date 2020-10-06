classdef planar_stereoCamModel < ObservationModelBase
    %PLANAR_STEREOCAMMODEL Camera model for a calibrated stereo camera
    %
    %   with features restricted to be on the plane, v value of features are
    %   all == v_max/2
    
    properties(Constant = true)
        obsDim = 2;
        obsNoiseDim = 2;
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
        FoV = deg2rad(50);
        eps = 1e-7;
    end
    
    methods
        function obj = planar_stereoCamModel(landmarkIDs, landmarkPoses)
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
            % idfs Ids of observations
                if nargin == 2 % noisy observations
                    noise = 1;  
                elseif nargin > 2 && strcmp('nonoise',varargin{1}) == 1 % nonoise
                    noise = 0;
                else                
                        error('unknown inputs')                
                end
                
                z = zeros(length(obj.landmarkIDs)*obj.obsDim,1);
                vis = ones(length(obj.landmarkIDs)*obj.obsDim,1);
                for j = 1:length(obj.landmarkIDs)
                   [z_j, vis_j] = obj.stereo_proj2D(obj.landmarkPoses(:,j), x);
                   if vis_j == 1
                        z(obj.obsDim*j-1: obj.obsDim*j) = z_j ...
                             + noise.*mvnrnd(zeros(obj.obsNoiseDim,1),diag([obj.var(1),obj.var(3)]),1).';
                   else
                       z(obj.obsDim*j-1: obj.obsDim*j) = z_j ...
                             + noise.*mvnrnd(zeros(obj.obsNoiseDim,1),diag([obj.var(1),obj.var(3)]),1).';
                       
                       vis(obj.obsDim*j-1: obj.obsDim*j) = vis_j;
                                       
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
            z_all = getObservation(obj, x);
            H_all = getObservationJacobian(obj, x,[]);
            z = z_all(z_all>0);
            H = H_all(z_all>0,:);
                
        end
        
        function [ y_j, isVisible ] = stereo_proj2D(obj, p_ji_i, x)
            %STEREO_PROJ Stereo projection model
            %inputs: p_ji_i - 2d point j in world frame
            %        x - robot state (x y theta)
            %outputs: y_j - stereo pixel coordinates, only u_left and
            %               u_right since planar
            th_vi = x(3);
            p_jc_i = p_ji_i - x(1:2);
            p_jc_i = planar_stereoCamModel.to3D(p_jc_i);
            C_vi = [cos(th_vi),sin(th_vi),0;
                    -sin(th_vi), cos(th_vi),0;
                    0, 0, 1;];
                
            C_ci = obj.C_cv*C_vi;
            
            p_jc_c = C_ci*p_jc_i;
            x = p_jc_c(1);
            y = p_jc_c(2);
            z = p_jc_c(3);

            y_j = zeros(2,1);

            y_j(1) = obj.f_u*x/z + obj.c_u;
            y_j(2) = obj.f_u*(x-obj.b)/z + obj.c_u;
            
            %Calculate visibility
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            if theta <= obj.FoV/2
                isVisible = 1;
            else
                isVisible = 0;
            end


        end
        
        function H = getObservationJacobian(obj, x,v)
            H = zeros(length(obj.landmarkIDs)*obj.obsDim,length(x));
            for j = 1:length(obj.landmarkIDs)
               H(obj.obsDim*j-1:obj.obsDim*j,:) = obj.stereoJac(obj.landmarkPoses(:,j), x);
            end 
        end
        
        function [ H_j ] = stereoJac(obj, p_ji_i, x_robot)
            %STEREOJAC Jacobian of stereo projection, needs 3 x 1 input point 
            %   Detailed explanation goes here
            th_vi = x_robot(3);
            %jacobian of transformation to feature in camera frame
            Z = [-cos(th_vi), -sin(th_vi), -sin(th_vi)*(p_ji_i(1)-x_robot(1)) + cos(th_vi)*(p_ji_i(2)-x_robot(2));
                  sin(th_vi), -cos(th_vi), -cos(th_vi)*(p_ji_i(1)-x_robot(1)) - sin(th_vi)*(p_ji_i(2)-x_robot(2));
                  0, 0, 0];
            
            %jacobian of projection to image plane
            C_vi = [cos(th_vi),sin(th_vi),0;
                    -sin(th_vi), cos(th_vi),0;
                    0, 0, 1;];
            T_vi = [C_vi, -C_vi*[x_robot(1:2);0];
                          zeros(1,3),1];
            T_cv = [obj.C_cv, zeros(3,1);
                        zeros(1,3),1];
            
            p_ji_i = planar_stereoCamModel.to4D(p_ji_i);
            p_jc_c = T_cv*T_vi*p_ji_i;
            x = p_jc_c(1);
            y = p_jc_c(2);
            z = p_jc_c(3);
            
            G_proj_ul = [obj.f_u/z,0,-obj.f_u*x/(z^2)];
            G_proj_ur = [obj.f_u/z,0,-obj.f_u*(x-obj.b)/(z^2)];
            G_proj = [G_proj_ul;G_proj_ur];
            
            H_j = G_proj*obj.C_cv*Z;

        end
        
        function p_jc_c = tf2camera_frame(obj,p_ji_i, x)
            th_vi = x(3);
            p_jc_i = p_ji_i - x(1:2);
            p_jc_i = planar_stereoCamModel.to3D(p_jc_i);
            C_vi = [cos(th_vi),sin(th_vi),0;
                    -sin(th_vi), cos(th_vi),0;
                    0, 0, 1;];
                
            C_ci = obj.C_cv*C_vi;
            
            p_jc_c = C_ci*p_jc_i;
            
        end
        
        function p_vis = visibility_probability(obj,p_jc_c,x)
            p_jc_c_norm = p_jc_c./norm(p_jc_c);
            theta = acos(p_jc_c_norm.'*[0;0;1]);
            
            if theta <= obj.FoV/2
                p_vis = 0.5*(cos(pi*theta/(obj.FoV/2)) + 1);
            else
                p_vis = 0;
            end

        end
        
        function M = getObservationNoiseJacobian(obj,x,v,z)
            n = length(z)/obj.obsDim;
            M = eye(n*obj.obsNoiseDim);
        end
        
        function R = getObservationNoiseCovariance(obj,x,z)
            
           variances_all_j = repmat([obj.var(1);obj.var(3)],length(obj.landmarkIDs),1);
           R = diag(variances_all_j);
           for j = 1:length(obj.landmarkIDs)
                p_ji_i = obj.landmarkPoses(:,j);
                p_jc_c = tf2camera_frame(obj,p_ji_i, x);
                p_vis_j = obj.visibility_probability(p_jc_c,x);
                R(j*obj.obsDim-1:j*obj.obsDim,j*obj.obsDim-1:j*obj.obsDim) =...
                        (1/(p_vis_j+obj.eps)).*R(j*obj.obsDim-1:j*obj.obsDim,j*obj.obsDim-1:j*obj.obsDim);
           end
           %Scale by visibility
           
            
        end
        
        function R_inv = getObservationNoiseCovarianceInverse(obj,x,z)
           variances_all_j = repmat([1/obj.var(1);1/obj.var(3)],length(obj.landmarkIDs),1);
           R_inv = diag(variances_all_j);
           for j = 1:length(obj.landmarkIDs)
                p_ji_i = obj.landmarkPoses(:,j);
                p_jc_c = tf2camera_frame(obj,p_ji_i, x);
                p_vis_j = obj.visibility_probability(p_jc_c,x);
                R_inv(j*obj.obsDim-1:j*obj.obsDim,j*obj.obsDim-1:j*obj.obsDim) =...
                        p_vis_j.*R_inv(j*obj.obsDim-1:j*obj.obsDim,j*obj.obsDim-1:j*obj.obsDim);
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
        

    end
end

%         function [ H_j ] = stereoJac(obj, p_ji_i, x_robot)
%             %STEREOJAC Jacobian of stereo projection, needs 3 x 1 input point 
%             %   Detailed explanation goes here
%             th_vi = x_robot(3);
%             C_vi = [cos(th_vi),sin(th_vi),0;
%                     -sin(th_vi), cos(th_vi),0;
%                     0, 0, 1;];
% %             C_ci = obj.C_cv*C_vi;
% %             r_ci_c = C_ci*[x_robot(1:2);0];
%             T_vi = [C_vi, -C_vi*[x_robot(1:2);0];
%                           zeros(1,3),1];
%             T_cv = [obj.C_cv, zeros(3,1);
%                         zeros(1,3),1];
%           
%                                   
%             D = [eye(3),zeros(3,1)];
%             p_ji_i = planar_stereoCamModel.to4D(p_ji_i);
%             p_jv_v = T_vi *p_ji_i;
%             p_jv_vdot = point2fs(p_jv_v);
%             Z = D*T_cv*p_jv_vdot;
%             
%             p_jc_c = T_cv*T_vi*p_ji_i;
%             
%             G_proj = zeros(4,3);
%             x = p_jc_c(1);
%             y = p_jc_c(2);
%             z = p_jc_c(3);
%             G_proj(1,:) = [obj.f_u/z, 0, -obj.f_u*x/(z^2)];
%             G_proj(2,:) = [0, obj.f_v/z, -obj.f_v*y/(z^2)];
%             G_proj(3,:) = [obj.f_u/z, 0, -obj.f_u*(x-obj.b)/(z^2)];
%             G_proj(4,:) = [0, obj.f_v/z, -obj.f_v*y/(z^2)];
%             H_j = G_proj*Z;
%             H_j = [H_j(1,1),H_j(1,2),H_j(1,6);
%                    H_j(3,1),H_j(3,2),H_j(3,6)];

