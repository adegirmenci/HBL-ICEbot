function success = cropImagesAndSave(studyDirPath, croppedFolderPath, cropMask) % studyName, studyRootPath, 
% CROPIMAGESANDSAVE: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 6/22/2016
%      Crops images found in folder 'studyDirPath'
%      Uses the mask 'cropMask'
%      Saves them to the folder 'croppedFolderPath'
% See also: ICEBOT_IMGPROCGUI_V1

% flag to return
success = false;

% get image file names
images = dir([studyDirPath,filesep, '*.jpg']);
fprintf('Found %d images in folder.\n',length(images))

% Load mask and cropping limits
% Cropping limits are stored in cropSettings.cropROI
% Cropping mask is stored in cropSettings.mask
load(['cropMasks',filesep,cropMask,'.mat']);
cropSettings.mask_uint8 = uint8(cropSettings.mask);

% make folder for cropped files
if ~exist(croppedFolderPath, 'dir')
  mkdir(croppedFolderPath);
end

nImages = length(images);
% Progress bar
hWait = waitbar(0,sprintf('Cropping image %d of %d',0,nImages));

% Iterate over images
for i = 1:nImages
    waitbar(i/nImages, hWait, sprintf('Cropping image %d of %d',i,nImages))
    
    fileName = images(i).name;
    
    % Load image to memory
    imgObj.imageOriginal = imread([studyDirPath,filesep,fileName]);

    % Get frame properties
    imgObj.imHeightOrig = size(imgObj.imageOriginal,1);
    imgObj.imWidthOrig = size(imgObj.imageOriginal,2);
    
    % Check to make sure mask size and image size fit
    if(cropSettings.imHeight ~= imgObj.imHeightOrig)
        error('Image height not compatible with mask!')
    end
    if(cropSettings.imWidth ~= imgObj.imWidthOrig)
        error('Image width not compatible with mask!')
        % disp(['[' 8 'Image width not compatible with mask!]' 8])
    end

    % Crop video
    imgObj.imageCropped = imcrop(imgObj.imageOriginal, cropSettings.cropROI);

    imgObj.imageProcessed = imgObj.imageCropped.*cropSettings.mask_uint8; %apply mask

    % Save as JPEG file
    fileNameOut = [croppedFolderPath,filesep,fileName(1:end-4),'.jp2'];

    imwrite(imgObj.imageProcessed, fileNameOut, 'jp2', 'Mode','lossless');
    
    fprintf('Save image to %s\n',fileNameOut)
    
end
close(hWait);

success = true;
