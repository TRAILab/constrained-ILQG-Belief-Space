function drawLandmarks(h, map)
%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Draw landmarks/beacons in the world.
% Input:
%   h: figure handle
%   landmarks: list of landmark vertices
%%%%%%%%%%%%%%%%%%%%%%%%%%%

figure(h)
hold on;

% phi = linspace(0,2*pi,50);
% 
% % draw white circle around markers
% for l = 1:size(landmarks,2)
%     
% %     scatter(landmarks(1,l),landmarks(2,l),2000*r,'filled', 'MarkerFaceAlpha',1/r^2,'MarkerFaceColor',[1.0 1.0 1.0]);
%     for r = 0:0.05:3        
%         plot(landmarks(1,l)+r*cos(phi),landmarks(2,l) + +r*sin(phi),...
%             'LineWidth',2,...
%             'Color',[1.0,1.0,1.0]./(1+r^2));           
%     end
% end

% put square in cente of beacon
if size(map.bounds,1) == 2
    plot(map.landmarks(1,:),map.landmarks(2,:),'ys',...
        'LineWidth',2,...
        'MarkerSize',8,...
        'MarkerFaceColor',[1.0,1.0,0.0])
else
    plot3(map.landmarks(1,:),map.landmarks(2,:),map.landmarks(3,:),'ys',...
        'LineWidth',2,...
        'MarkerSize',8,...
        'MarkerFaceColor',[1.0,1.0,0.0])
end

end