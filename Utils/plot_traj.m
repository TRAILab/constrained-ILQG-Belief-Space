function plot_traj(b_nom, b_traj_t, roboTraj,dt)
%PLOT_TRAJ Plot belief trajectories and input trajectories
%   Detailed explanation goes here
    %%  Plotting
t = dt.*[1:1:length(b_nom)];
figure
subplot(3,1,1)
%Plot nominal belief and 3sig bound
plot(t,b_nom(1,:) - roboTraj(1,:),'r-')
hold on
plot(t,3*sqrt(b_nom(4,:)),'r--')
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
legend({'$\tilde{e}_x$','nominal $3\sigma$ bounds','$\hat{e}_x$','actual $3\sigma$ bounds'},'Interpreter','Latex')


% %Plot true state
% plot(t,roboTraj(1,:),'k-')
% legend({'$\tilde{x}$','nominal $3\sigma$ bounds','$\hat{x}$','actual $3\sigma$ bounds','${x}$'},'Interpreter','Latex')
xlabel('Time, (s)','Interpreter','Latex')
ylabel('$x$-position error, (m)','Interpreter','Latex')
% %xlim([0 1261])
%%
subplot(3,1,2)
%Plot nominal belief and 3sig bound
plot(t,b_nom(2,:) - roboTraj(2,:),'r-')
hold on
plot(t,3*sqrt(b_nom(7,:)),'r--')
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
legend({'$\tilde{e}_y$','nominal $3\sigma$ bounds','$\hat{e}_y$','actual $3\sigma$ bounds'},'Interpreter','Latex')
%xlim([0 1261])
xlabel('Time, (s)','Interpreter','Latex')
ylabel('$y$-position error, (m)','Interpreter','Latex')
%%
subplot(3,1,3)
%Plot nominal belief and 3sig bound
plot(t,wrapToPi(b_nom(3,:) - roboTraj(3,:)),'r-')
hold on
plot(t,3*sqrt(b_nom(9,:)),'r--')
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
legend({'$\tilde{e}_x$','nominal $3\sigma$ bounds','$\hat{e}_x$','actual $3\sigma$ bounds'},'Interpreter','Latex')
%xlim([0 1261])
xlabel('Time, (s)','Interpreter','Latex')
ylabel('Heading error $\theta$, (rad)','Interpreter','Latex')


end

