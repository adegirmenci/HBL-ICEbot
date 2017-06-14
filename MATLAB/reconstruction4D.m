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
                   'sweepIdx',[],... % sweep that the image belongs to
                   'errorPsi',[],... % angular error from desired pose
                   'ECGcycleIdx',[],... % ECG cycle that the image belongs to
                   'percentECG',[]); % location in the ECG cycle, value b/w 0.0 and 1.0

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

%% Load ECG

[handles.ECG_time, handles.ECGvoltage] = ...
    importECG([handles.ECGpathName,handles.ECGfileName]);

if(~isempty(handles.ECG_time))
    set(handles.ECGgatingButton, 'Enable','on');
    axes(handles.axes1);
    plot(handles.ECG_time, handles.ECGvoltage);
else
    set(handles.ECGgatingButton, 'Enable','off');
end

msg = sprintf('Loaded %d ECG readings.\n', length(handles.ECG_time));
set(handles.numECGtextbox, 'String', msg);

%% ECG Gating

minPeakDist = 60/100/2;
minPeakHei = 0.2;

[handles.ECGpeakLocs, handles.ECGpeakTime, avgHR, stdHR] = ...
    ECGgating(handles.ECGvoltage, handles.ECG_time, handles.axes1, minPeakDist, minPeakHei);

msg = sprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);

%% Align ECG with images

[allImgs, reorderedImgs, accurateImgs, nSweeps] = EM_ECG_matching(handles);

handles.allImages = allImgs;
handles.reorderedImgs = reorderedImgs;
handles.accurateImgs = accurateImgs;
handles.nSweeps = nSweeps;

%% Load crop mask

cropMask = 'Acuson_Epiphan.mat';
% cropMask = 'Sequoia_StarTech_Size 1 (largest).mat';
load(['.',filesep,'cropMasks',filesep,cropMask]);
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

% find bin with max count
[maxBinCount,maxBinIdx] = max(sum(logical(binCounts),1));
fprintf('%d out of %d sweeps align well at bin number %d\n', maxBinCount,nSweeps,maxBinIdx);

goodSweepIdx = binCounts > 0;
nSlicesToStitch4D = sum(goodSweepIdx,1);


% find the most common time by making a vector of ECG pctg 
% then do a most likelihood estimate

listOfECGs = cell(n4Dframes,1);
phat = zeros(n4Dframes,2);
for i = 1:n4Dframes
    nSlicesToStich = nSlicesToStitch4D(i);
    
    sub_goodSweepIdx = find(goodSweepIdx(:,i));
    
    for j = 1:nSlicesToStich
        currSweepIdx = sub_goodSweepIdx(j);
        inSweepIdx = sweepIdx == currSweepIdx; % in this sweep
        ecgs = percentECG(inSweepIdx);
        
        idxs = (ecgs < binMaxs(i)) & (ecgs >= binMins(i));
        listOfECGs{i} = [listOfECGs{i}, ecgs(idxs)];
    end
    phat(i,:) = mle(listOfECGs{i});
    fprintf(['Frame %d: Mean ECG phase is: %.3f ', char(177) ' %.3f\n'], i, phat(i,1), phat(i,2));
end

% stitch
interpCube = [];
dt = datestr(datetime('now'),'_yymmdd_HHMMss');
for k = 1:n4Dframes
    nSlicesToStitch = nSlicesToStitch4D(k);
    sub_goodSweepIdx = find(goodSweepIdx(:,k));
    
    for i = 1:nSlicesToStitch
        currSweepIdx = sub_goodSweepIdx(i);
        inSweepIdx = sweepIdx == currSweepIdx; % in this sweep
        ecgs = percentECG(inSweepIdx);
        
        isItTrue = (ecgs < binMaxs(k)) & (ecgs > binMins(k));
        ecgvals = ecgs(isItTrue);
        diff_ = abs(ecgvals - phat(k,1));
        [~,imIdx] = min(diff_);
        
        theseImgs = allImages(inSweepIdx);
        theseImgs = theseImgs(isItTrue);
        fileName = theseImgs(imIdx).FileName;
        
        reportECG = theseImgs(imIdx).percentECG;
        fprintf('ECG: %.2f\n', reportECG);
        
        % Load image to memory
        fullName = [handles.studyDirPath,filesep,fileName];
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
        
        stitch.originalEM{i} = theseImgs(imIdx).poseEM;
        
        imshow(stitch.imageCropped{i});
        drawnow
    end

stitch.imHeightOrig = size(cropSettings.mask,1);
stitch.imWidthOrig = size(cropSettings.mask,2);

%%%%% need respiration compensation / tissue based tracking

% initialize container 
stitch.imageLocXYZval = cell(nSlicesToStitch,1);
stitch.nFrames = nSlicesToStitch;

% interpolate
[CdZeroed,interpCube] = interpolateSlices4D(stitch, cropMask, 'true', interpCube);

outfilePre = ['volume_',handles.studyName,dt,'_',num2str(k)];

% save as .RAW
volumeFileName = [outfilePre,'.raw'];
saveStitched2RawFile(CdZeroed, volumeFileName)

% save as .mat
save([outfilePre,'.mat'],'CdZeroed')

% save volume size to text file
[xn,yn,zn] = size(CdZeroed);

fileID = fopen([outfilePre,'.txt'],'w');
fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
fclose(fileID);

end