%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% Demo for a 2D belief space planning scenario with a
% point robot whose body is modeled as a disk
% and has a heading direction. A stereo camera is attached to the robot,
% with the camera axis aligned with the heading
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

function main
close all
clear all
clc
% add subfolders to path
% addpath(genpath('./'));
fname = 'with_normals2';
% clean up
% clear variables; clc; close all; dbstop if error;

fprintf('\n A demonstration of the iLQG algorithm for Belief Space Planning \n')

% Whether or not to store data to file
CREATE_OUTPUT_DIRECTORY = 1; % set to 1 for writing output

% number of sims to run
NUM_SIMS = 1;

% which map to use
% fname = 'mapTask3';

% create full path to map name
mapFilePath = strcat('./Maps/',fname,'.mat');

time = clock; % Gets the current time as a 6 element vector

newFolderName = 'uni_iLQG_30_FoV_nonsmooth2';   

% Lets check for platform type i.e., Windows, Linux and define base folder
% accordingly base diretory where runs live
if isunix ==1
    [~,username] = system('whoami');
    baseDirectory = ['/home/',username(1:end-1),'/Research_code/bsp-ilqg-master/Results'];
    % Mac is unix so have to check here
    if ismac==1
        baseDirectory = ['/Users/',username(1:end-1),'/Documents/MATLAB/TRO/SingleHomotopy/'];
    end
end

% Create new directory
if CREATE_OUTPUT_DIRECTORY
    fstat = mkdir(baseDirectory,newFolderName);
    
    % if unsuccessful, exit
    if fstat == 0
        error('Could not create directory to save files');
    end
    
end

% path to where data is written
outDatPath = strcat(baseDirectory,newFolderName,'/');

for i = 1:NUM_SIMS
    
    if CREATE_OUTPUT_DIRECTORY
        mkdir(outDatPath,['run',num2str(i)]);
    end
    
    plan_unicycle_robot(mapFilePath, [outDatPath,'run',num2str(i),'/']);
%     plan_quadPlane(mapFilePath, [outDatPath,'run',num2str(i),'/']);
end
end
