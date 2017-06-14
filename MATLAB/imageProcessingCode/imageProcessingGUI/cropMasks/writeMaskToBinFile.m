close all; clear all; clc

% load mask
load('Acuson_Epiphan.mat');

% open file
fileID = fopen('Acuson_Epiphan.bin','w');

%write width (int)
fwrite(fileID, int32(cropSettings.imWidth), 'int32');

%write height (int)
fwrite(fileID, int32(cropSettings.imHeight), 'int32');

% write crop ROI (int)x4
fwrite(fileID, int32(cropSettings.cropROI), 'int32');

% write mask
m = cropSettings.mask';
fwrite(fileID, uint8(m(:))*255, 'uint8');

fclose(fileID);