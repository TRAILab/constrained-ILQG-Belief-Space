function drawObstacles(h,map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw obstacles in the world. 
% Input:
% h: figure handle
% obstacles: list of obstacle vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;

if size(map.bounds,1) == 2
    for i = 1:size(map.obstacles,2)
        R = map.obstacle_sizes(i);
        obs = map.obstacles(:,i);
        lp = obs - [R;R];
        pos = [lp' R R];
        p = rectangle('Position',pos','FaceColor',[0.66, 0.84, 0.92]);
        p.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end
else
    for i = 1:size(map.obstacles,2)
        R = map.obstacle_sizes(i);
        obs = map.obstacles(:,i);
        lp = obs - [R;R];
        pos = [lp',0];
        plotcube([R,R,R],pos,1,[0.66, 0.84, 0.92]);
%         p.Annotation.LegendInformation.IconDisplayStyle = 'off';
    end

end