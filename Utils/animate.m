function [failed, b_traj_t, roboTraj,trCov_vs_time, u_actual] = animate(figh, plotFn, b0, b_nom, u_nom, L, motionModel, obsModel, stateValidityChecker, map, DYNAMIC_OBS)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Animate the robot's motion from start to goal
%
% Inputs:
%   figh: Figure handle in which to draw
%   plotfn: function handle to plot cov ellipse
%   b0: initial belief
%   b_nom: nominal belief trajectory
%   u_nom: nominal controls
%   L: feedback gain
%   motionModel: robot motion model
%   obsModel: observation model
% Outputs:
% failed: 0 for no collision, 1 for collision, 2 for dynamic obstacle
% detected
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
stDim = motionModel.stDim;

xt = b0(1:stDim); % true state of robot
x = b0(1:stDim); % estimated mean

b_traj_t = zeros(length(b0),length(b_nom(1,:)));
b_traj_t(:,1) = b0;
u_actual = zeros(size(u_nom));

% unpack covariance from belief vector
D = motionModel.D;
vecP = D*b0(stDim+1:end);
P = reshape(vecP,stDim,stDim); % Covariance Matrix

rh = []; % robot disk drawing handle

figure(figh);
handle = plot(b_nom(1,:),b_nom(2,:),'r', 'LineWidth',2);


% create robot body points
global ROBOT_RADIUS
robotDisk = ROBOT_RADIUS*[cos(linspace(0,2*pi,50));...
    sin(linspace(0,2*pi,50))];

trCov_vs_time(1) = trace(P);

roboTraj = zeros(stDim,length(b_nom(1,:)));
roboTraj(:,1) = xt;
distSegToObs = zeros(1, size(map.obstacles(1,:),2));
distSegToObs(:) = 30; % set to a large number
distTrajToObs = zeros(1, length(b_nom(1,:))-1);
trajLength = 0;

failed = 0;
handles = [];

for i = 1:size(u_nom,2)
    
    if DYNAMIC_OBS == 1
        if stateValidityChecker(b_nom(1:2,min(i+3,size(b_nom,2)))) == 0
            figure(figh);
            plot(roboTraj(1,:),roboTraj(2,:),'g', 'LineWidth',2);          
            drawnow;
            warning('Robot expected to Collide with dynamic obstacle!!!');
            failed = 2;
            return;
        end
    end
    
    b = [x(:);motionModel.D_pseudoinv*P(:)]; % current belief
    
    u = u_nom(:,i) + L(:,:,i)*(b - b_nom(:,i));
    u_actual(:,i) = u;
    
    % update robot
    processNoise = motionModel.generateProcessNoise(xt,u); % process noise
    xt = motionModel.evolve(xt,u,processNoise);
    %xt = motionModel.evolve(xt,u,motionModel.zeroNoise);
    
    % Get motion model jacobians and predict pose
    zeroProcessNoise = motionModel.zeroNoise; % process noise
    x_prd = motionModel.evolve(x,u,zeroProcessNoise); % predict robot pose
    A = motionModel.getStateTransitionJacobian(x,u,zeroProcessNoise);
    G = motionModel.getProcessNoiseJacobian(x,u,zeroProcessNoise);
    Q = motionModel.getProcessNoiseCovariance(x,u);
    
    % Get observation model jacobians
    %z = obsModel.getObservation(xt,'nonoise'); % true observation
    [z,vis] = obsModel.getObservation(xt); % true observation
    z_prd = obsModel.getObservation(x_prd,'nonoise'); % predicted observation
    z_prd = z_prd(vis==1); % Only visible features
    H = obsModel.getObservationJacobian(x_prd,[]);
    H = H(vis==1,:); % Only visible features
    z = z(vis==1); % Only visible features
    M = obsModel.getObservationNoiseJacobian(x,[],z);
    R = obsModel.getObservationNoiseCovarianceReal(x,z);
    M = M(vis==1,vis==1);
    R = R(vis==1,vis==1);
    
    % update P
    P_prd = A*P*A' + G*Q*G';
    
    if length(z) == 0
        x = x_prd;
        P = P_prd;
    else
        S = H*P_prd*H' + M*R*M';
        K = (P_prd*H')/S;
        P = (eye(stDim) - K*H)*P_prd;
        x = x_prd + K*(z - z_prd);
    end
    
    % final belief
    b_f = [x;motionModel.D_pseudoinv*P(:)];
    b_traj_t(:,i+1) = b_f;
    
    roboTraj(:,i+1) = xt;
    
    trCov_vs_time(i+1) = trace(P);

    % calculate distance to closest obstacle, and increment path length
    segLen = norm(xt - roboTraj(:,i));
    trajLength = trajLength + segLen;
    for obs_count = 1:size(map.obstacles(1,:),2)
        if segLen == 0.0
            projection = xt;
        else
            t_line = dot((map.obstacles(:,obs_count)-0.5*map.obstacle_sizes(obs_count)) - roboTraj(1:2,i), ...
                            xt(1:2) - roboTraj(1:2,i)) ...
                    / (segLen*segLen);
            t_seg = max([0 min([1 t_line])]);
            projection = roboTraj(1:2,i) + t_seg * (xt(1:2) - roboTraj(1:2,i));
        end
        distSegToObs(obs_count) = norm((map.obstacles(:,obs_count) - projection)-0.5*map.obstacle_sizes(obs_count)) - (sqrt(2)/2)*map.obstacle_sizes(obs_count) - ROBOT_RADIUS; % To simplify calculation, still treat as circle obstacle
    end
    distTrajToObs(i) = min(distSegToObs);
    distSegToObs(:) = 30;
    
    % if robot is in collision
    if stateValidityChecker(xt) == 0
        figure(figh);
        plot(roboTraj(1,1:i+1),roboTraj(2,1:i+1),'g', 'LineWidth',2);
        drawnow;
        warning('Robot collided :( ');
        failed = 1;
        return;
    end

    delete(rh)
    rh = fill(xt(1) + robotDisk(1,:),xt(2) + robotDisk(2,:),'b');
    set(get(get(rh,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
    handles = drawFoV(figh,obsModel,xt,handles);
    drawResult(plotFn,b_f,length(b(:,1)),motionModel.D);
    drawnow;
    legend({'Features','Start','Goal','Nominal rolled out trajectory'},'Interpreter','Latex')
    pause(0.05);
end

constr_values = constraintFunc(b_traj_t(:,2:end), u_actual, 1:size(u_nom,2));
fprintf(['\n'...
        'max constraint violation: %-12.7g\n' ...
        'min distance to obstacle: %-12.7g\n' ...
        'trajectory length: %-12.7g\n'], ...
        max(constr_values), min(distTrajToObs), trajLength);

figure(figh);
sg = plot(roboTraj(1,:),roboTraj(2,:),'g', 'LineWidth',2);
set(get(get(sg,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
drawnow;
failed = 0;
end