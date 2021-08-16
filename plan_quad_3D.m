%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a belief space planning scenario in 3D world with a
% non holonomic car like robot whose body is modeled as a disk
% and has a heading direction. A stereo camera is attached to the robot on
% a pan-tilt gimbal
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plan_quad_3D(mapPath, trainPath, outDatPath)

close all;

%% Initialize planning scenario
DYNAMIC_OBS = 0;

dt = 0.1; % time step
% control_method = 'iLQG';
control_method = 'iLQG_AL';

load(mapPath); % load map
[~,map_name,~] = fileparts(mapPath);

mm = quadrotorPlanar(dt); % motion model

om = stereoCamModel(1:length(map.landmarks(1,:)),map.landmarks); % observation model

global ROBOT_RADIUS;
ROBOT_RADIUS = 0.16; % robot radius is needed by collision checker

svc = @(x)isStateValid(x,map,0); % state validity checker (collision)

%% Setup start and goal/target state
map.start = [3;1;deg2rad(90)];
map.goal = [8;6;deg2rad(-90)];

% map.start = [5;0.5;deg2rad(0)];
% map.goal = [13;5.0;deg2rad(45)];

x0 = map.start; % intial state
P = 0.01*eye(3); % intial covariance
b0 = [x0;mm.D_pseudoinv*P(:)]; % initial belief state

xf = map.goal; % target state

%% Setup planner to get nominal controls

[x_traj0,u0] = initial_rollout_quad(map,mm,[],40);
nDT = size(u0,2); % Time steps

%% set up the optimization problem

% Set full_DDP=true to compute 2nd order derivatives of the
% dynamics. This will make iterations more expensive, but
% final convergence will be much faster (quadratic)
full_DDP = false;
% conFunc = @(b,u,k) constraintFunc(b,u,k);

del_r = 0.05;
conFunc = @(b,u,k) chance_constr(b,u,k,map,mm.stDim,mm.D,del_r);
Op.no_constr = length(map.obstacles(1,:));
Op.phi = repmat(0.01,Op.no_constr,1);
if strcmp(control_method, 'iLQG_AL')
    % these function is needed by iLQG_AL
    xy_cstr_bound = 0.13;
    ang_cstr_bound = 0.1 ; % temporary fix here since anonymous function call cannot return multiple values, write the x direction constraint
    DYNCST  = @(b,u,lagMultiplier, mu,k) beliefDynCostConstr(b,u,lagMultiplier, mu,k,xf,nDT,full_DDP,mm,om,svc,conFunc,map); % For iLQG_AL
elseif strcmp(control_method, 'iLQG')
    info_cost = 2000; % temporary fix here since anonymous function call cannot return multiple values, write the parameter of Q_t
    DYNCST  = @(b,u,i) beliefDynCost(b,u,xf,nDT,full_DDP,mm,om,svc,map); % For iLQG
%     DYNCST  = @(b,u,i) beliefDynCostFIF(b,u,xf,nDT,full_DDP,mm,om,svc,map); % Use FIF based dynamics
%     DYNCST  = @(b,u,i) beliefDynCost_nonsmooth(b,u,xf,nDT,full_DDP,mm,om,svc,map); % For iLQG without visibility smoothing
end   

% control constraints are optional
% Op.lims  = [-1.0 1.0;         % V forward limits (m/s)
%     -1.0  1.0];        % angular velocity limits (m/s)
% Op.lims  = mm.ctrlLim;        % Vy limits (m/s)
Op.lims = [];        % Vy limits (m/s)

Op.plot = 1; % plot the derivatives as well

%% prepare the visualization window and graphics callback
figh = figure;
%set(figh,'WindowStyle','docked');
drawLandmarks(figh,map);
drawObstacles(figh,map);
scatter(x0(1),x0(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[1.0 0.0 0.0])
scatter(xf(1),xf(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[0.0 1.0 0.0])
legend({'Start','Goal','Mean trajectory with covariance ellipses'},'Interpreter','Latex')
set(gcf,'name','Belief Space Planning with iLQG','NumberT','off');
% set(gca,'Color',[0.0 0.0 0.0]);
% set(gca,'xlim',map.bounds(1,[1,2]),'ylim',map.bounds(2,[1,2]),'zlim',map.bounds(3,[1,2]),'DataAspectRatio',[1 1 1])
 set(gca,'xlim',[0,9],'ylim',[0,10],'zlim',[0,3],'DataAspectRatio',[1 1 1])
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
box on

%% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','r','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));

legend({'Features','Start','Goal','Mean trajectory with covariance ellipses'},'Interpreter','Latex')
Op.plotFn = plotFn;
Op.D = mm.D;
Op.om = om;


%% === run the optimization
if strcmp(control_method, 'iLQG')
    training_file_name = strcat(control_method, '_cost_', num2str(info_cost,3), '_map', map_name, '.mat');
%     training_file_name = strcat(control_method, '_cost_', num2str(info_cost,3), '_map', map_name, 'FIF.mat');
elseif strcmp(control_method, 'iLQG_AL')
    training_file_name = strcat(control_method, '_cstr_', num2str(xy_cstr_bound,3), '_', num2str(ang_cstr_bound,3), '_map_', map_name, '.mat');
end
training_file_path = strcat(trainPath, training_file_name);
if isfile(training_file_path)
    fprintf('Training file found: %s', training_file_name)
    load(training_file_path);
    b = results.b;
    u_opt = results.u_opt;
    L_opt = results.L_opt;
    tt = results.tt;
    nIter = results.nIter;
    optimCost = results.optimCost;
    trace = results.trace;
    drawResult(plotFn, b,length(b(:,1)),mm.D)
else
    if strcmp(control_method, 'iLQG')
        [b,u_opt,L_opt,~,~,optimCost,trace,~,tt, nIter]= iLQG(DYNCST, b0, u0, Op);
    elseif strcmp(control_method, 'iLQG_AL')
        [b,u_opt,L_opt,~,~,optimCost,trace,~,tt, nIter]= iLQG_AL(DYNCST, b0, u0, Op);
    end
end

drawFoV(om,b(1:3,:),[],1);


%% Save result figure
try
%     savefig(figh,strcat(outDatPath,'iLQG-solution'));
%     savefig(initGuessFigure,strcat(outDatPath,'RRT-initGuess'));
catch ME
    warning('Could not save figs')
end

results.mmNoiseSigma = sqrt(diag(mm.P_Wg));
% results.omNoiseSigma = om.sigma_b;
results.optimCost = fliplr(cumsum(fliplr(optimCost)));
results.b = b;
results.u_opt = u_opt;
results.L_opt = L_opt;
results.tt = tt;
results.nIter = nIter;
results.x0 = x0;
results.xf = xf;
results.trace = trace;
[c,viol] = constraint_checker(b,u_opt,1e-3);
violations = nnz(viol);
cost_traj = sum(costFunc_wo_unc(b(:,2:end), u_opt, xf, size(b,2), 3, svc, map, Op.D, 1));

%% plot the final trajectory and covariances

svcDyn = @(x)isStateValidAnimate(x,map,DYNAMIC_OBS); % state validity checker (collision)
% % 
[didCollide, b_actual_traj, x_traj_true,trCov_vs_time{1},u_actual_traj] = animate(figh, plotFn, b0, b, u_opt, L_opt, mm, om, svcDyn, map, DYNAMIC_OBS);
% 
plot_traj(b, b_actual_traj, x_traj_true, dt, conFunc, outDatPath) % Plot belief errors
% 
% results.collision{1} = didCollide;
% 
% 
% try
%     savefig(figh,strcat(outDatPath,'iLQG-1'));
% catch ME
%     warning('Could not save figs')
% end
% 
% 
if ~isfile(training_file_path)
    try
        save(training_file_path, 'results');
    catch ME
        warning('Could not save training result')
    end
end

end
