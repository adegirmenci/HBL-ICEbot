close all; clear all; clc

% Written by Alperen Degirmenci
% 6/14/2017 - Harvard Biorobotics Lab

%% Select Study Directory

root = 'C:\Users\Alperen\Documents\QT Projects\ICEbot_QT_v1\LoggedData\';
folder = uigetdir(root);

study = folder(length(root)+1:end);

%% Load Controller Data

[CycleNum,Time,Type,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16] = ...
  importControllerFile([folder,filesep,study,'_Controller.txt']);

numRec = length(Time);

%% Extract DXYZPSI

mask = strcmp(Type,'DXYZPSI');

relevantIdx = find(mask);
relevantTime = (Time(relevantIdx) - Time(1))./1000.0;

numRelevant = length(relevantIdx);

dxyzpsi = [x1(relevantIdx), x2(relevantIdx), x3(relevantIdx), x4(relevantIdx)];

%% Extract T_BB_CT

mask = strcmp(Type,'T_BB_CT');
if(~isempty(mask))
    resetbbIdx = find(strcmp(Type,'RESETBB'));
    resetbbIdx = resetbbIdx(end);
    
    relevantIdx = find(mask);
    relevantIdx = relevantIdx(relevantIdx>resetbbIdx);
    Time_T_BB_CT = Time(relevantIdx);
    
    % convert to posix time
    hrsOffet = 5;
    if(isdst(datetime('today','TimeZone','America/New_York')))
        hrsOffet = 4;
    end
    Time_T_BB_CT = posixtime(datetime(str2num(study(1:4)),str2num(study(5:6)),str2num(study(7:8)))...
                             + hours(hrsOffet) + milliseconds(Time_T_BB_CT));
    
    numRelevant = length(relevantIdx);
    
    T_BB_CT = zeros(4,4,length(Time_T_BB_CT));
    T_BB_CT(1,1,:) = x1(relevantIdx);
    T_BB_CT(2,1,:) = x2(relevantIdx);
    T_BB_CT(3,1,:) = x3(relevantIdx);
    T_BB_CT(4,1,:) = x4(relevantIdx);
    T_BB_CT(1,2,:) = x5(relevantIdx);
    T_BB_CT(2,2,:) = x6(relevantIdx);
    T_BB_CT(3,2,:) = x7(relevantIdx);
    T_BB_CT(4,2,:) = x8(relevantIdx);
    T_BB_CT(1,3,:) = x9(relevantIdx);
    T_BB_CT(2,3,:) = x10(relevantIdx);
    T_BB_CT(3,3,:) = x11(relevantIdx);
    T_BB_CT(4,3,:) = x12(relevantIdx);
    T_BB_CT(1,4,:) = x13(relevantIdx);
    T_BB_CT(2,4,:) = x14(relevantIdx);
    T_BB_CT(3,4,:) = x15(relevantIdx);
    T_BB_CT(4,4,:) = x16(relevantIdx);
else
    disp('No T_BB_CT in file. Must be older format.');
end

%% Import Images

[imageFileNames,imageTimestamps] = importImageTimestamps([folder,filesep,study,'_FrmGrab.txt']);

allImages = struct('FileName',imageFileNames,... % file name of image
                   'TimeStamp',num2cell(imageTimestamps),... % time stamp of image
                   'poseEM',[],... % interpolated pose
                   'EMidx',[],... % index of closest EM reading
                   'errorPsi',[]); % angular error from desired pose

%% Align image and EM timestamps

for i = 1:length(imageTimestamps)
    [~,idx] = min(abs(Time_T_BB_CT*1000.0 - imageTimestamps(i)));
    closestEM = T_BB_CT(:,:,idx);
    allImages(i).poseEM = closestEM;
end

figure
plot(Time_T_BB_CT, zeros(length(Time_T_BB_CT),1))
hold on;
plot(imageTimestamps/1000.,zeros(length(imageTimestamps),1),'o')
figure

%% Load crop mask

load(['.',filesep,'cropMasks',filesep,'Acuson_Epiphan.mat']);
%load(['.',filesep,'cropMasks',filesep,'Sequoia_StarTech_Size 1 (largest).mat']);
% gives us a variable named cropSettings
cropSettings.mask_uint8 = uint8(cropSettings.mask);


%% Interpolate images

n4Dframes = 15;

binCounts = zeros(nSweeps,n4Dframes);
temp = linspace(0,1,n4Dframes+1);
binMins = temp(1:end-1);
binMaxs = temp(2:end);
% find the best aligned ECG spot
for i = 1:nSweeps
    inSweepIdx = (sweepIdx == i); % in this sweep
    ecgs = percentECG(inSweepIdx);
    for j = 1:n4Dframes
        isItTrue = (ecgs < binMaxs(j)) & (ecgs > binMins(j));
        binCounts(i,j) = sum(isItTrue);
    end
end

nSlicesToStitch = length(imageTimestamps);

% stitch
for i = 1:nSlicesToStitch
    imIdx = i;
    
    fileName = allImages(imIdx).FileName;

    % Load image to memory
    fullName = [folder,filesep,'images',filesep,fileName];
    %fprintf('%s\n',fullName);
    stitch.imageOriginal{i} = imread(fullName);
  
    imH = size(stitch.imageOriginal{i},1);
    imW = size(stitch.imageOriginal{i},2);
    if(i == 1)
        % get frame size
        stitch.imHeightOrig = imH;
        stitch.imWidthOrig = imW;
    else
        % check image size
        if(~(stitch.imHeightOrig == imH) || ~(stitch.imWidthOrig == imW) )
            error('Images have different sizes!')
        end
    end
    % Check to make sure mask size and image size fit
    if(cropSettings.imHeight ~= stitch.imHeightOrig)
        error('Image height not compatible with mask!')
    end
    if(cropSettings.imWidth ~= stitch.imWidthOrig)
        error('Image width not compatible with mask!')
        % disp(['[' 8 'Image width not compatible with mask!]' 8])
    end
    
    stitch.imageCropped{i} = imcrop(stitch.imageOriginal{i}, cropSettings.cropROI);

    stitch.imageCropped{i} = stitch.imageCropped{i}.*cropSettings.mask_uint8; %apply mask
    
    stitch.originalEM{i} = allImages(imIdx).poseEM;
    
    imshow(stitch.imageCropped{i});
    drawnow
end
stitch.imHeightOrig = size(cropSettings.mask,1);
stitch.imWidthOrig = size(cropSettings.mask,2);

%% initialize container 
stitch.imageLocXYZval = cell(nSlicesToStitch,1);
stitch.nFrames = nSlicesToStitch;

%% interpolate
[CdZeroed] = interpolateSlices(stitch, 'Acuson_Epiphan', true');

dt = datestr(datetime('now'),'_yymmdd_HHMMss');
outfilePre = ['.',filesep,'volumes',filesep,'volume_',study(2:end),dt];

%% save as .RAW
volumeFileName = [outfilePre,'_1.raw'];
saveStitched2RawFile(CdZeroed, volumeFileName)

%% save as .mat
save([outfilePre,'.mat'],'CdZeroed')

%% save volume size to text file
[xn,yn,zn] = size(CdZeroed);

fileID = fopen([outfilePre,'.txt'],'w');
fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
fclose(fileID);

fprintf('Done!\n')

%% if acquiring images once at a time, at each target

% import tip data

% import images

% align image timestamp with EM and select the pertinent readings

% reconstruct


%% else, if acquiring images continuously

% import tip data

% import images

% align image timestamps with EM, put readings into memory

% figure out when the angle between images is greater than a threshold

% reconstruct