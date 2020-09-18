function [x_traj,u_traj,figh] = initial_rollout_unicycle(map, motion_model, waypoints, timesteps)
%INITIAL_ROLLOUT Computes a nominal control and state trajectory based on
%   hand crafted waypoints
%   Detailed explanation goes here
    if isempty(waypoints)
        waypoints = [map.start, [5;8.5;deg2rad(52)], [8;8.5;deg2rad(-35)], [9;2.5;deg2rad(-48)], [14;2.5;deg2rad(58)], [14;5;deg2rad(120)], map.goal];
    end
    x_traj = [];
    u_traj = [];
    for k = 2:length(waypoints(1,:))
        x_k_minus = waypoints(:,k-1);
        x_k = waypoints(:,k);
        
        nom_traj = motion_model.generate_open_loop_point2point_traj(x_k_minus,x_k);
        x_traj = [x_traj, nom_traj.x];
        u_traj = [u_traj, nom_traj.u];
        
    end
    
    % Apply inputs to dynamics
    for k = 1:length(u_traj(1,:))
        x_traj(:,k+1) = motion_model.evolve(x_traj(:,k),u_traj(:,k),motion_model.zeroNoise);
    end
    
    % Draw
    
    
    figh = figure();
    drawObstacles(figh,map)
    drawLandmarks(figh,map)
    plot(x_traj(1,:),x_traj(2,:),'r-o')
end
