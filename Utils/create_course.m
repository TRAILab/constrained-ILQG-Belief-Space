function [map] = create_course(save_map)
%CREATE_COURSE Summary of this function goes here
%   Detailed explanation goes here

    xlim = [0,14];
    ylim = [0,10];

    map.bounds = [xlim;ylim];
    map.obstacles = [2,2;
                 2,3;
                 2.5,5;
                 3,7;
                 3.5,9;
                 4,11;
                 6,11;
                 8,11;   
                 10,12;
                 12,11;
                 12,9;
                 12,7;
                 12,5;
                 13,3;
                 14,2;
                 9,1.5;
                 7,2;
                 7,3;
                 7,4;
                 7,5;
                 7,6;
                 7,7;
                 7,8]';
             map.obstacle_sizes = [2,1,2,2,2,2,2,2,2,2,2,2,2,1,2,1,1,1,1,1,1,1,1];

    map.landmarks = [2,2;
                     2,3;
                     2.5,3;
                     2.5,5;
                     3,5;
                     3,7;
                     3.5,7;
                     3.5,9;
                     8,9;
                     8,10;
                     10,10;
                     10,3;
                     12,3;
                     6,7;
                     7,7;
                     7,1;].';
    map.start = [3;1;deg2rad(90)];
    map.goal = [9;2;wrapTo2Pi(-1.4056)];
    
    sPath = '/home/wavelab/Research_code/bsp-ilqg-master/Maps/course2dhard2.mat';

    if save_map == 1
        save(sPath,'map');
    end

fprintf('Done \n');
end

