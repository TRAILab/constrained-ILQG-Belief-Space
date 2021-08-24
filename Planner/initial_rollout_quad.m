function [x_traj,u_traj,figh] = initial_rollout_quad(map,motion_model, waypoints, timesteps)
%INITIAL_ROLLOUT Computes a nominal control and state trajectory based on
%   hand crafted waypoints
%   Detailed explanation goes here
    if isempty(waypoints)
       goal = map.goal;
       goal(3) = deg2rad(90);
       waypoints = [map.start, [5;8.5;deg2rad(90)],  map.goal];
%        waypoints = [map.start, [5;8.5;deg2rad(52)],  [8;8.5;deg2rad(-35)], map.goal];
%        waypoints = [map.start, [8;1.0;deg2rad(0)], [8;2.5;deg2rad(0)], [11;2.0;deg2rad(-45)],...
%                                     [14;3.0;deg2rad(-90)] map.goal];
    end
    x_k_minus = waypoints(:,1);
    x_traj = x_k_minus;
    u_traj = [];
    for k = 2:length(waypoints(1,:))
        x_k = waypoints(:,k);        
        nom_traj = motion_model.generate_open_loop_point2point_traj(x_k_minus,x_k,floor(timesteps/length(waypoints(1,:))));
        x_traj = [x_traj, nom_traj.x(:,2:end)];
        u_traj = [u_traj, nom_traj.u];
        x_k_minus = x_traj(:,end);
        
    end
    
    % Draw
    
    %x_traj = x_traj(:,1:59);
    %u_traj = u_traj(:,1:59);
    
%     timesteps_remaining = timesteps - length(x_traj(1,:));
%     x_traj = [x_traj, repmat(x_traj(:,end),1,timesteps_remaining)];
%     u_traj = [u_traj, zeros(motion_model.ctDim,timesteps_remaining)];
    
%     figh = figure();
%     drawObstacles(figh,map)
%     drawLandmarks(figh,map)
%     plot(x_traj(1,:),x_traj(2,:),'r-o')
end
