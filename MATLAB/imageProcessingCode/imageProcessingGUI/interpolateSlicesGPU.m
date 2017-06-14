function [CdZeroed] = interpolateSlicesGPU(stitch, cropMask, doPadding)
% INTERPOLATESLICESGPU: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 7/14/2016
%      Given a set of 2DUS images, crops, aligns, and interpolates them
%      into a 3DUS volume. Accelerate using GPU.
% See also: ICEBOT_IMGPROCGUI_V1

hWait = waitbar(0, 'Interpolating slices...');

% load mask
load(['cropMasks',filesep,cropMask,'.mat']);

waitbar(0.1, hWait, 'Loaded crop mask.');

% calculate pixel size
stitch.usPlaneLength = 76.6-3.9; %76.0; % mm
stitch.pixSize = stitch.usPlaneLength/stitch.imHeightOrig; % mm/pix
% generate image to EM transformation matrix
stitch.T_CT_IMG = [0 1 0 0;...
            0 0 -1 0;...
           -1 0 0 stitch.imWidthOrig/2.0*stitch.pixSize;...
            0 0 0 1];

% grid
[X,Y] = meshgrid(0:stitch.imWidthOrig-1,0:stitch.imHeightOrig-1); % mesh of img idx

Xvec = X(:);
Yvec = Y(:);
maskVec = logical(cropSettings.mask(:));
XvecNo0 = Xvec(maskVec);
YvecNo0 = Yvec(maskVec);
NpixelsNo0 = sum(maskVec);

j = 1;
for i = 1:stitch.nFrames
    % prepare container
    stitch.imageLocXYZval{j} = zeros([NpixelsNo0,4]); % initialize N-by-4
    stitch.imageLocXYZval{j}(:,1) = XvecNo0*stitch.pixSize; % scale to mm
    stitch.imageLocXYZval{j}(:,2) = YvecNo0*stitch.pixSize; % scale to mm
    % Z is zero
    tmp = stitch.imageCropped{j};
    % insert image
    tmp = tmp(:);
    stitch.imageLocXYZval{j}(:,4) = tmp(maskVec);
    
    % register images based on EM
    tmp = stitch.imageLocXYZval{j}';
    tmp(4,:) = 1; % put it in [x;y;z;1] format
    tmp = stitch.originalEM{i}*stitch.T_CT_IMG*tmp;
    tmp = tmp';
    stitch.imageLocXYZval{j}(:,1:3) = tmp(:,1:3);
    j = j + 1;
end
waitbar(0.4, hWait, 'Transformed to image coordinates.');

xyz = zeros([NpixelsNo0*stitch.nFrames,3]);
c = zeros([NpixelsNo0*stitch.nFrames,1]);

for j = 1:stitch.nFrames
    xyz((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,1:3);
    c((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,4);
end
waitbar(0.5, hWait, 'Extracted points.');

if(doPadding)
    % pad either side with same image
    xyzPadPlus = zeros([NpixelsNo0*stitch.nFrames,3]);
    xyzPadMinus = zeros([NpixelsNo0*stitch.nFrames,3]);
    for j = 1:stitch.nFrames
        % find normal -> y axis
        tmp = stitch.originalEM{i}*stitch.T_CT_IMG;
        yDir = tmp(1:3,2)';
        offsetDist = 0.5; % pixel
        xyzPadPlus((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,1:3) + repmat(yDir*offsetDist,size(stitch.imageLocXYZval{j},1),1);
        xyzPadMinus((j-1)*NpixelsNo0+1:j*NpixelsNo0,:) = stitch.imageLocXYZval{j}(:,1:3) - repmat(yDir*offsetDist,size(stitch.imageLocXYZval{j},1),1);
    end
    waitbar(0.5, hWait, 'Padding done.');
end

observations = [xyz,c; xyzPadPlus,c; xyzPadMinus,c];
volume = interpolateAvg_v2(observations,0.9);
waitbar(0.75, hWait, 'Discretized points.');

tic
Cd = inpaintn(volume,100);
toc
waitbar(0.95, hWait, 'Interpolation done.');

CdZeroed = Cd;
CdZeroed(isnan(CdZeroed)) = 0;

close(hWait)

end