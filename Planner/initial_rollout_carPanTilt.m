function [x_traj,u_traj] = initial_rollout_carPanTilt(map, motion_model, waypoints, timesteps)
%INITIAL_ROLLOUT Computes a nominal control and state trajectory based on
%   hand crafted waypoints
%   Detailed explanation goes here
    if isempty(waypoints)
        waypoints = [map.start, [5;8.5;deg2rad(52);0;0],  map.goal];
%                waypoints = [map.start, [8;1.0;deg2rad(90);0;0], [8;2.5;deg2rad(0);0;0], [11;2.0;deg2rad(0);0;0],...
%                                     [14;2.0;deg2rad(90);0;0],[14;4.0;deg2rad(180);0;0]];
    end
    
    x_k_minus = map.start(1:3);
    x_traj = x_k_minus;
    u_traj = [];
    for k = 2:length(waypoints(1,:))
        x_k = waypoints(1:3,k);        
        nom_traj = motion_model.generate_open_loop_point2point_traj(x_k_minus,x_k);
        x_traj = [x_traj, nom_traj.x(:,2:end)];
        u_traj = [u_traj, nom_traj.u];
        x_k_minus = x_traj(:,end);
    end
    
    % Apply inputs to dynamics
    x_traj = [x_traj;zeros(2, length(x_traj(1,:)))];
    u_traj = [u_traj;zeros(2, length(u_traj(1,:)))];
    for k = 1:length(u_traj(1,:))
        x_traj(:,k+1) = motion_model.evolve(x_traj(:,k),u_traj(:,k),motion_model.zeroNoise);
    end
    
    % Draw
    
    
%     figh = figure();
%     drawObstacles(figh,map)
%     drawLandmarks(figh,map)
%     plot(x_traj(1,:),x_traj(2,:),'r-o')
end
