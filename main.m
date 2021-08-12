%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D belief space planning scenario with a
% point robot whose body is modeled as a disk
% and has a heading direction. A stereo camera is attached to the robot,
% with the camera axis aligned with the heading
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main
% close all
clear all
clc

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

% Whether or not to store data to file
CREATE_OUTPUT_DIRECTORY = 1; % set to 1 for writing output

% number of sims to run
NUM_SIMS = 1;

% which map to use

fname = 'gap_left';

% create full path to map name
mapFilePath = strcat('./Maps/',fname,'.mat');

time = fix(clock); % Gets the current time as a 6 element vector
timeStamp = [num2str(time(1)), num2str(time(2),'%02.f'), num2str(time(3),'%02.f'),'_',... % year, month, day
    num2str(time(4),'%02.f'), num2str(time(5),'%02.f'), num2str(time(6),'%02.f')]; % hour, minute, second
 
robotType = 'quad';
% robotType = 'uni';
% robotType = 'car';
envSettingFolderName = strcat(robotType, '_iLQG_2');

% Lets check for platform type i.e., Windows, Linux and define base folder
% accordingly base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    % baseDirectory = ['/home/',username(1:end-1),'/Research_code/bsp-ilqg-master/Results'];
    baseDirectory = './Results/';
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/TRO/SingleHomotopy/'];
    end
end

% path to where data is written
outDatPath = strcat(baseDirectory, envSettingFolderName, '/');
trainPath = strcat('./Results_FIFTest/', envSettingFolderName, '/');

% Create new directory
if CREATE_OUTPUT_DIRECTORY
    if ~exist(outDatPath, 'dir')
        fstat = mkdir(baseDirectory, envSettingFolderName);
        % if unsuccessful, exit
        if fstat == 0
            error('Could not create directory to save files');
        end
    end

    if ~exist(trainPath, 'dir')
        fstat = mkdir(trainPath);
    end
end

for i = 1:NUM_SIMS
    
    if CREATE_OUTPUT_DIRECTORY
        mkdir(outDatPath, [timeStamp,'_run',num2str(i)]);
    end
    
    outDatPathFull = strcat(outDatPath, timeStamp, '_run', num2str(i), '/');
    if strcmp(robotType, 'uni')
        plan_unicycle_robot(mapFilePath, trainPath, outDatPathFull);
    elseif strcmp(robotType, 'quad')
%         plan_quadPlane(mapFilePath, trainPath, outDatPathFull);
        plan_quad_3D(mapFilePath, trainPath, outDatPathFull);
    elseif strcmp(robotType, 'car')
        plan_car_3D(mapFilePath, trainPath, outDatPathFull)
    end
    
end
end
