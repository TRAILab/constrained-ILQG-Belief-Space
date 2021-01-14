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

    % Landmarks are [x_pos,y_pos, direction]
    % Direction of landmarks is in degrees and measured from world x-axis, used for detecting parallax
    % error while feature matching
    map.landmarks = [
%                      2,3,1,-45;
%                      2.5,3,2,0;
%                      2.5,5,2,-45;
%                      3,5,2,0;
%                      3,7,2,-45;
%                      3.5,7,2,0;
%                      3.5,9,2,-45;
%                      4,9,2,-90;
%                      6,9,2,-90;
%                      8,9,2,-90;
%                      8,9,-90;
%                      8,10,-45;
%                      10,10,225;
 %                    10,3,1,180;
                     6,1,1,-135;
                     6,8,1,135;
                     7,8,1,45;
                     7,1,1,45;
%                      8,1.5,135;
                     9,1.5,1,45;
                     13,3,0,-45;
                     13,4,0,45;
                     10,3,0,-135;
                     12,1,2,90;
                     14,1,2,135;
                     15,6,0,-135
                     14,6,2,-90;
                     10,3,1,-135;
                     13,4,1,45].';
%     map.start = [3;1;deg2rad(90),0,0];
%     map.goal = [12;5;wrapTo2Pi(-1.4056),0,0];
    
    % sPath = '/home/wavelab/Research_code/bsp-ilqg-master/Maps/course2dhard2.mat';
    sPath = 'Maps/3D_3.mat';

    if save_map == 1
        save(sPath,'map');
    end

fprintf('Done \n');
end

