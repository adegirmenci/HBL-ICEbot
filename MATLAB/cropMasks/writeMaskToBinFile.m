close all; clear all; clc

maskName = 'Sequoia_StarTech_Size 1 (largest)80mm partial';

% load mask
load([maskName,'.mat']);


% open file
fileID = fopen([maskName,'.bin'],'w');

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