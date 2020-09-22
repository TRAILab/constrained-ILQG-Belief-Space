%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D belief space planning scenario with a
% point robot whose body is modeled as a disk
% and has a heading direction. A stereo camera is attached to the robot,
% with the camera axis aligned with the heading
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
function plan_unicycle_robot(mapPath, outDatPath)

close all;

%% Initialize planning scenario
DYNAMIC_OBS = 0;

dt = 0.1; % time step
results_path = '/home/wavelab/Research_code/bsp-ilqg-master/Resultsuni_iLQG_30_FoV_smooth/run1/ilqg_results_1.mat';
load(mapPath); % load map

mm = unicycle_robot(dt); % motion model

om = planar_stereoCamModel(1:length(map.landmarks(1,:)),map.landmarks); % observation model

global ROBOT_RADIUS;
ROBOT_RADIUS = 0.16; % robot radius is needed by collision checker

svc = @(x)isStateValid(x,map,0); % state validity checker (collision)

%% Setup start and goal/target state

x0 = map.start; % intial state
P = 0.02*eye(3); % intial covariance
% sqrtSigma0 = sqrtm(Sigma0);
b0 = [x0;mm.D_psuedoinv*P(:)]; % initial belief state

xf = map.goal; % target state

%% Setup planner to get nominal controls

[x_traj0,u0, initGuessFigure] = initial_rollout_unicycle(map,mm,[],85);
nDT = size(u0,2); % Time steps

%% set up the optimization problem

% Set full_DDP=true to compute 2nd order derivatives of the
% dynamics. This will make iterations more expensive, but
% final convergence will be much faster (quadratic)
full_DDP = false;

% these function is needed by iLQG_AL
conFunc = @(b,u,k) constraintFunc(b,u,k);
% DYNCST  = @(b,u,lagMultiplier, mu,k) beliefDynCostConstr(b,u,lagMultiplier, mu,k,xf,nDT,full_DDP,mm,om,svc,conFunc,map); % For iLQG_AL
% DYNCST  = @(b,u,i) beliefDynCost(b,u,xf,nDT,full_DDP,mm,om,svc,map); % For iLQG
DYNCST  = @(b,u,i) beliefDynCost_nonsmooth(b,u,xf,nDT,full_DDP,mm,om,svc,map); % For iLQG without visibility smoothing

% control constraints are optional
% Op.lims  = [-1.0 1.0;         % V forward limits (m/s)
%     -1.0  1.0];        % angular velocity limits (m/s)
% Op.lims  = mm.ctrlLim;        % Vy limits (m/s)
Op.lims  = [];        % Vy limits (m/s)

Op.plot = 1; % plot the derivatives as well

%% prepare the visualization window and graphics callback
figh = figure;
set(figh,'WindowStyle','docked');
drawLandmarks(figh,map);
drawObstacles(figh,map);
scatter(x0(1),x0(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[1.0 0.0 0.0])
scatter(xf(1),xf(2),250,'filled','MarkerFaceAlpha',1/2,'MarkerFaceColor',[0.0 1.0 0.0])
legend({'Start','Goal','Mean trajectory with covariance ellipses'},'Interpreter','Latex')
set(gcf,'name','Belief Space Planning with iLQG','NumberT','off');
set(gca,'Color',[0.0 0.0 0.0]);
set(gca,'xlim',map.bounds(1,[1,2]),'ylim',map.bounds(2,[1,2]),'DataAspectRatio',[1 1 1])
xlabel('X (m)'); ylabel('Y (m)');
box on

%% prepare and install trajectory visualization callback
line_handle = line([0 0],[0 0],'color','r','linewidth',2);
plotFn = @(x) set(line_handle,'Xdata',x(1,:),'Ydata',x(2,:));

% legend({'Features','Start','Goal'},'Interpreter','Latex')
legend({'Features','Start','Goal','Mean trajectory with covariance ellipses'},'Interpreter','Latex')
Op.plotFn = plotFn;
Op.D = mm.D;

%% === run the optimization

load_controller = 0;
if load_controller
    load(results_path);
    b = results.b;
    u_opt = results.u_opt;
    L_opt = results.L_opt;
    tt = results.tt;
    nIter = results.nIter;
    optimCost = results.optimCost;
    trace = results.trace;
    drawResult(plotFn, b, mm.stDim,mm.D)
else
    [b,u_opt,L_opt,~,~,optimCost,trace,~,tt, nIter]= iLQG(DYNCST, b0, u0, Op);
%     [b,u_opt,L_opt,~,~,optimCost,trace,~,tt, nIter]= iLQG_AL(DYNCST, b0, u0, Op);
end
fprintf(['sum of u:   %-12.7g\n'],norm(u_opt));

rh = [];
lh = [];
for k = 1:length(b(1,:))
    x_mean = b(1:3,k);
    drawFoV(figh,om,x_mean,rh,lh);
    pause(0.1)
end



%% Save result figure
try
    savefig(figh,strcat(outDatPath,'iLQG-solution'));
    savefig(initGuessFigure,strcat(outDatPath,'RRT-initGuess'));
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

%% plot the final trajectory and covariances
if DYNAMIC_OBS == 1
    drawObstacles(figh,map.dynamicObs);
end

svcDyn = @(x)isStateValidAnimate(x,map,DYNAMIC_OBS); % state validity checker (collision)


[didCollide, b_actual_traj, x_traj_true,trCov_vs_time{1}, u_actual] = animate(figh, plotFn, b0, b, u_opt, L_opt, mm, om, svcDyn, map,DYNAMIC_OBS);

plot_traj(b, b_actual_traj, x_traj_true, dt, conFunc, outDatPath) % Plot belief errors

results.collision{1} = didCollide;

try
    savefig(figh,strcat(outDatPath,'iLQG-A2B-soution'));
catch ME
    warning('Could not save figs')
end


try
    save(strcat(outDatPath,'ilqg_results_1.mat'), 'results');
catch ME
    warning('Could not save results')
end

end


