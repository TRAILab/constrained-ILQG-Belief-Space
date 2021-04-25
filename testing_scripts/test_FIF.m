%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Testing script for Fisher Information Field implementation
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
clear all
close all
clc

load('./Maps/3D_1.mat');
load('./Results_RAL2/quad_iLQG_2/iLQG_AL_cstr_0.13_0.1_map_3D_1.mat');

om = bearingSensor(1:length(map.landmarks(1,:)),map.landmarks);
v_alpha = 0.5;
om.set_vis_coeffs(v_alpha);
x0 = [3;1;deg2rad(90)];

traj = results.b(1:3,:);

%% prepare the visualization window and graphics callback
figh = figure;
%set(figh,'WindowStyle','docked');
drawLandmarks(figh,map);
drawObstacles(figh,map);
scatter(x0(1),x0(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[1.0 0.0 0.0])
set(gca,'xlim',map.bounds(1,[1,2]),'ylim',map.bounds(2,[1,2]),'zlim',map.bounds(3,[1,2]),'DataAspectRatio',[1 1 1])
% set(gca,'xlim',[0,10],'ylim',[0,10],'zlim',[0,3],'DataAspectRatio',[1 1 1])
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
box on
drawFoV(om,traj,[],3);

error_FIF_avg = 0;
error_vis_prob_avg = 0;
t_FIF_avg = 0;
t_vis_prob_avg = 0;

for k = 1:length(traj(1,:))
    %% Get measurements, real analytical and fd Jacobians, measurement info
    x = traj(:,k);
    [z_real, vis_real] = om.getObservation(x,'nonoise');

    del_x = eye(3).*(1e-10);
    x1 = repmat(x,1,3) + del_x;

    f2 = [om.getObservation(x1(:,1),'nonoise'),om.getObservation(x1(:,2),'nonoise'),...
                om.getObservation(x1(:,3),'nonoise')];
    fd = (f2 - repmat(z_real,1,3))./1e-10;
    analytical_jac = om.getObservationJacobian(x,[]);

    %% Compare actual and rotation invariant FIM
    true_jac = analytical_jac(vis_real==1,:);
    true_FIM = (true_jac.'*true_jac)./om.bear_var;
    vis_real = reshape(vis_real,3,15);
    rot_inv_FIM = zeros(3,3);
    for j = 1:length(vis_real(1,:))
        if(vis_real(1,j)) == 1
            p_jc_i = om.landmarkPoses(1:3,j) - [x(1:2);om.height];
            rot_inv_FIM = rot_inv_FIM + om.compute_FIM_actual(p_jc_i);
        end
    end

    %% Build one FIF voxel
    C_I = om.compute_FIM_pos(x);
    tic
    V_I = om.compute_FIM_rot(x);
    FIF_FIM = V_I*C_I;
    t_FIF = toc;
    
    t_FIF_avg = t_FIF_avg + (t_FIF - t_FIF_avg)/k;

    %% Compare with Probabilistic Visibility model
    tic
    R_inv = om.getObservationNoiseCovarianceInverse(x0,[]);
    prob_vis_FIM = analytical_jac.'*R_inv*analytical_jac;
    t_prob_vis = toc;
    t_vis_prob_avg = t_vis_prob_avg + (t_prob_vis - t_vis_prob_avg)/k;

    error_FIF = norm(true_FIM - FIF_FIM)/norm(true_FIM);
    error_FIF_avg = error_FIF_avg + (error_FIF - error_FIF_avg)/k;
    
    error_vis_prob = norm(true_FIM - prob_vis_FIM)/norm(true_FIM);
    error_vis_prob_avg = error_vis_prob_avg + (error_vis_prob - error_vis_prob_avg)/k;
    
end