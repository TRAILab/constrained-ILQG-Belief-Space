function [x_traj,u_traj,figh] = initial_rollout_unicycle(map, motion_model, waypoints, timesteps)
%INITIAL_ROLLOUT Computes a nominal control and state trajectory based on
%   hand crafted waypoints
%   Detailed explanation goes here
    if isempty(waypoints)
        % waypoints = [map.start, [5;8;deg2rad(74)], [8;8;wrapTo2Pi(-1.2056)],map.goal];
        waypoints = [map.start, [5;8.5;deg2rad(74)], [8;8.5;wrapTo2Pi(-1.2056)], [9;2.5;deg2rad(-45)], [14;2.5;deg2rad(45)], [14;5;deg2rad(135)], map.goal];
    end
    x_traj = [];
    u_traj = [];
    for k = 2:length(waypoints(1,:))
        x_k_minus = waypoints(:,k-1);
        x_k = waypoints(:,k);
        
        nom_traj = motion_model.generate_open_loop_point2point_traj(x_k_minus,x_k,floor(timesteps/length(waypoints(1,:))));
        x_traj = [x_traj, nom_traj.x];
        u_traj = [u_traj, nom_traj.u];
        
    end
    
    % Draw
    
    % x_traj = x_traj(:,1:59);
    % u_traj = u_traj(:,1:59);
    
    % timesteps_remaining = timesteps - length(x_traj(1,:));
    % x_traj = [x_traj, repmat(x_traj(:,end),1,timesteps_remaining)];
    % u_traj = [u_traj, zeros(motion_model.ctDim,timesteps_remaining)];
    
    figh = figure();
    drawObstacles(figh,map)
    drawLandmarks(figh,map)
    plot(x_traj(1,:),x_traj(2,:),'r-')
end

