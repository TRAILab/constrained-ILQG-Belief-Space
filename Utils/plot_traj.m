function plot_traj(b_nom, b_traj_t, roboTraj,dt, constr_func, outDatPath)
%PLOT_TRAJ Plot belief trajectories and input trajectories
%   Detailed explanation goes here
    %%  Plotting
    
% Retrieve Constraint values
constr_val = 3.*sqrt(-constr_func(zeros(length(b_nom(:,1)),1),0,5));

t = dt.*[1:1:length(b_nom)];
figh = figure;
subplot(3,1,1)
%Plot nominal belief and 3sig bound
% plot(t,b_nom(1,:) - roboTraj(1,:),'r-')

plot(t,3*sqrt(b_nom(4,:)),'r--')
hold on
ph = plot(t,-3*sqrt(b_nom(4,:)),'r--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds'},'Interpreter','Latex')

%Plot actual belief and 3sig bound
plot(t,b_traj_t(1,:) - roboTraj(1,:),'b-')
hold on
plot(t,3*sqrt(b_traj_t(4,:)),'b--')
ph = plot(t,-3*sqrt(b_traj_t(4,:)),'b--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds','$\hat{x}$','$3 actual \sigma$ bounds'},'Interpreter','Latex')

% Plot constraints
plot([t(1),t(end)],[constr_val(1),constr_val(1)],'m--');
ph = plot([t(1),t(end)],[-constr_val(1),-constr_val(1)],'m--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
legend({'nominal $3\sigma$ bounds','$\hat{e}_x$','actual $3\sigma$ bounds', 'Constraint on $3\sigma$ bound'},'Interpreter','Latex')




% %Plot true state
% plot(t,roboTraj(1,:),'k-')
% legend({'$\tilde{x}$','nominal $3\sigma$ bounds','$\hat{x}$','actual $3\sigma$ bounds','${x}$'},'Interpreter','Latex')
xlabel('Time, (s)','Interpreter','Latex')
ylabel('$x$-position error, (m)','Interpreter','Latex')
% %xlim([0 1261])
%%
subplot(3,1,2)
%Plot nominal belief and 3sig bound
% plot(t,b_nom(2,:) - roboTraj(2,:),'r-')

plot(t,3*sqrt(b_nom(7,:)),'r--')
hold on
ph = plot(t,-3*sqrt(b_nom(7,:)),'r--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds'},'Interpreter','Latex')

%Plot actual belief and 3sig bound
plot(t,b_traj_t(2,:) - roboTraj(2,:),'b-')
hold on
plot(t,3*sqrt(b_traj_t(7,:)),'b--')
ph = plot(t,-3*sqrt(b_traj_t(7,:)),'b--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds','$\hat{x}$','$3 actual \sigma$ bounds'},'Interpreter','Latex')

% Plot constraints
plot([t(1),t(end)],[constr_val(2),constr_val(2)],'m--');
ph = plot([t(1),t(end)],[-constr_val(2),-constr_val(2)],'m--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

legend({'nominal $3\sigma$ bounds','$\hat{e}_y$','actual $3\sigma$ bounds','Constraint on $3\sigma$ bound'},'Interpreter','Latex')
%xlim([0 1261])
xlabel('Time, (s)','Interpreter','Latex')
ylabel('$y$-position error, (m)','Interpreter','Latex')
%%
subplot(3,1,3)
%Plot nominal belief and 3sig bound
% plot(t,wrapToPi(b_nom(3,:) - roboTraj(3,:)),'r-')

plot(t,3*sqrt(b_nom(9,:)),'r--')
hold on
ph = plot(t,-3*sqrt(b_nom(9,:)),'r--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds'},'Interpreter','Latex')

%Plot actual belief and 3sig bound
plot(t,wrapToPi(b_traj_t(3,:) - roboTraj(3,:)),'b-')
hold on
plot(t,3*sqrt(b_traj_t(9,:)),'b--')
ph = plot(t,-3*sqrt(b_traj_t(9,:)),'b--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');
% legend({'$\tilde{x}$','$3 nominal \sigma$ bounds','$\hat{x}$','$3 actual \sigma$ bounds'},'Interpreter','Latex')

% Plot constraints
plot([t(1),t(end)],[constr_val(3),constr_val(3)],'m--');
ph = plot([t(1),t(end)],[-constr_val(3),-constr_val(3)],'m--');
set(get(get(ph,'Annotation'),'LegendInformation'),'IconDisplayStyle','off');

legend({'nominal $3\sigma$ bounds','$\hat{e}_x$','actual $3\sigma$ bounds','Constraint on $3\sigma$ bound'},'Interpreter','Latex')
%xlim([0 1261])
xlabel('Time, (s)','Interpreter','Latex')
ylabel('Heading error $\theta$, (rad)','Interpreter','Latex')

try
    savefig(figh,strcat(outDatPath,'belief-errors'));
catch ME
    warning('Could not save figs')
end

end

