function [map] = create_3D_course(save_map)
%CREATE_COURSE Summary of this function goes here
%   Detailed explanation goes here

    xlim = [0,16];
    ylim = [0,10];
    zlim = [0,5];

    map.bounds = [xlim;ylim;zlim];
    map.obstacles = [2,2;
                 2,3;
                 2.5,5;
                 3,7;
                 3.5,9;
                 4,11;
                 6,11;
                 8,11;   
                 10,12;
                 12,10;
                 12,8;
%                  11,6;
%                  11,5;
                 11,4;
                 12,4;
                 13,4;
                 14,8;
                 16,8;
                 17,6;
                 17,4;
                 16,2;
                 14,1;
                 10,1.5;
                 7,2;
                 7,3;
                 7,4;
                 7,5;
                 7,6;
                 7,7;
                 7,8]';
             map.obstacle_sizes = [2,1,2,2,2,2,2,2,2, 2,2,1,1,1, 2,2,2,2,2,2, 1,1,1,1,1,1,1,1];

    % Landmarks are [x_pos,y_pos, z_pos, viewing_direction]
    % Direction of landmarks is in degrees and measured from world x-axis, used for detecting parallax
    % error while feature matching
    map.landmarks = [
                     2,3,1,-45;
                     2.1,3,2,-45;
                     2.2,3,2,-45;
                     2.3,3,2,-45;
                     2.4,3,2,-45;
                     2.5,3,2,-45;
                     2.5,3.2,2,-45;
                     2.5,3.4,2,-45;
                     2.5,3.6,2,-45;
                     2.5,3.8,2,-45;
                     2.5,4.0,2,-45;
                     2.5,4.4,2,-45;
                     2.5,4.6,2,-45;
                     2.5,4.8,2,-45;
                     2.5,5,2,-45;
                     2.6,5,2,-45;
                     2.7,5,2,-45;
                     2.8,5,2,-45;                     
                     3,5.2,2,0;
                     3,5.4,2,0;
                     3,5.6,2,0;
                     3,5.8,2,0;
                     3,6.0,2,0;
                     3,6.2,2,0;
                     3,6.4,2,0;
                     3,6.6,2,0;
                     3,6.8,2,0; 
                     3,7,2,-45;
                     3.5,7,2,0;
                     3.5,7.2,2,0;
                     3.5,7.4,2,0;
                     3.5,7.6,2,0;
                     3.5,7.8,2,0;
                     3.5,8,2,0;
                     3.5,8.2,2,0;
                     3.5,8.4,2,0;
                     3.5,8.6,2,0;
                     3.5,8.8,2,0;  
                     3.5,9,2,-45;
                     3.6,9,2,-60;
                     3.7,9,2,-60;
                     3.8,9,2,-60;
                     3.9,9,2,-60;
                     4,9,2,-90;
                     4.2,9,2,-90;
                     4.4,9,2,-90;
                     4.6,9,2,-90;
                     4.8,9,2,-90;
                     5.0,9,2,-90;
                     5.2,9,2,-90;
                     5.4,9,2,-90;
                     5.6,9,2,-90;
                     5.8,9,2,-90;
                     6,9,2,-90;
                     6.0,9,2,-90;
                     6.2,9,2,-90;
                     6.4,9,2,-90;
                     6.6,9,2,-90;
                     6.8,9,2,-90;
                     7.0,9,2,-90;
                     7.2,9,2,-90;
                     7.4,9,2,-90;
                     7.6,9,2,-90;
                     7.8,9,2,-90;
                     8,9,2,-90;
                     ].';

    
    % sPath = '/home/wavelab/Research_code/bsp-ilqg-master/Maps/course2dhard2.mat';
    sPath = 'Maps/3D_many_left.mat';
    
%     figh = figure;
%     drawLandmarks(figh,map);
%     drawObstacles(figh,map);
%     set(gca,'xlim',map.bounds(1,[1,2]),'ylim',map.bounds(2,[1,2]),'zlim',map.bounds(3,[1,2]),'DataAspectRatio',[1 1 1]);

    if save_map == 1
        save(sPath,'map');
    end

fprintf('Done \n');
end


