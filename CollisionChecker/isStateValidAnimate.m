function [yesno,c2c_padded] = isStateValidAnimate(x, map, dynamicObs)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Check if robot is in collision with obstacles, used before animation only
% Input:
%   x: robot state
%   map: obstacle map
%   varargin: robot radius to override default
%
% Output:
%   yesno: 1 if robot is not in collision (valid state)
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% robot radius
global ROBOT_RADIUS;
R = ROBOT_RADIUS/2;

N = 50; % discretize robot body
delta_theta = 2*pi/N;
theta = 0:delta_theta:2*pi-delta_theta;

% robot perimeter
ptPerimiter = repmat(x(1:2),1,N) + R*[cos(theta);sin(theta)] ;

% % check if robot is within boundary
bounds_xv = [map.bounds(1,1),map.bounds(1,2),map.bounds(1,2),map.bounds(1,1)] ;
bounds_yv = [map.bounds(2,1),map.bounds(2,1),map.bounds(2,2),map.bounds(2,2)];
inbounds = inpolygon(ptPerimiter(1,:),ptPerimiter(2,:),bounds_xv,bounds_yv);

% if robot not within bounds return false
if sum(inbounds) ~= size(ptPerimiter,2)
    yesno = 0;
    return;
end

% calculate distances of robot to obstacle centers
dx = map.obstacles(1,:)-0.5.*map.obstacle_sizes - x(1);
dy = map.obstacles(2,:)-0.5.*map.obstacle_sizes - x(2);

c2c_padded = sqrt(dx.^2 + dy.^2) - (sqrt(2)/2).*map.obstacle_sizes - R;
c2c_padded = c2c_padded.';

% more precise collision calculation
if any(c2c_padded <= 0)
    collided_index = find(c2c_padded <= 0);
    for i = collided_index
        if x(1) >= map.obstacles(1,i)-map.obstacle_sizes(i)-R & x(1) <= map.obstacles(1,i)+R & ...
           x(2) >= map.obstacles(2,i)-map.obstacle_sizes(i)-R & x(2) <= map.obstacles(2,i)+R % still not a perfect statement
            yesno = 0;
            return;
        end
    end
end

if dynamicObs == 1
    dx = map.dynamicObstacles(1,:) - x(1);
    dy = map.dynamicObstacles(2,:) - x(2);
    
    c2c_padded = sqrt(dx.^2 + dy.^2);
    
    if any(c2c_padded <= R + map.obstacleRadius)
        yesno = 0;
        return;
    end
end

yesno = 1;

end
