function drawObstacles(h,map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw obstacles in the world. 
% Input:
% h: figure handle
% obstacles: list of obstacle vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;


for i = 1:size(map.obstacles,2)
    R = map.obstacle_sizes(i);
    obs = map.obstacles(:,i);
    lp = obs - [R;R];
    pos = [lp' R R];
    rectangle('Position',pos','FaceColor','m');
end

end