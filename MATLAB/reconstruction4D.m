close all; clear all; clc

% Written by Alperen Degirmenci
% 6/14/2017 - Harvard Biorobotics Lab

cropMask = 'Sequoia_StarTech_Size 1 (largest)80mm';
% cropMask = 'Sequoia_StarTech_Size 1 (largest)80mm partial';

%% Select Study Directory

if(ispc)
%     root = 'C:\Users\Alperen\Documents\QT Projects\ICEbot_QT_v1\LoggedData\';
    root = 'D:\BIDMC_Exp5\';
elseif(ismac)
    root = '/Volumes/Macintosh HD/Research/BIDMC Exp 5/BIDMC_Exp5/';
else
    error('This script does not work on your system.');
end
folder = uigetdir(root);

study = folder(length(root)+1:end);

%% Load Controller Data

[CycleNum,Time,Type,x1,x2,x3,x4,x5,x6,x7,x8,x9,x10,x11,x12,x13,x14,x15,x16] = ...
  importControllerFile([folder,filesep,study,'_Controller.txt']);

numRec = length(Time);

% convert to posix time
YearMonthDay = [str2num(study(1:4)),str2num(study(5:6)),str2num(study(7:8))];
hrsOffet = 5;
if(isdst(datetime(YearMonthDay,'TimeZone','America/New_York')))
    hrsOffet = 4;
end
Time = posixtime(datetime(YearMonthDay)...
                          + hours(hrsOffet) + milliseconds(Time));

%Time = Time - Time(1);

%% Extract DXYZPSI

mask = strcmp(Type,'DXYZPSI');

relevantIdx = find(mask);
%relevantTime = (Time(relevantIdx) - Time(1))./1000.0;
relevantTime = Time(relevantIdx);

numRelevant = length(relevantIdx);

dxyzpsi = [CycleNum(relevantIdx), relevantTime, x1(relevantIdx), x2(relevantIdx), x3(relevantIdx), x4(relevantIdx)];

%% Extract T_BB_CT

mask = strcmp(Type,'T_BB_CT');
if(~isempty(mask))
    resetbbIdx = find(strcmp(Type,'RESETBB'));
    resetbbIdx = resetbbIdx(end);
    
    relevantIdx = find(mask);
    relevantIdx = relevantIdx(relevantIdx>resetbbIdx);
    Time_T_BB_CT = Time(relevantIdx);
    
    numRelevant = length(relevantIdx);
    
    T_BB_CT_cycleNum = CycleNum(relevantIdx);
    
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
imageTimestamps = imageTimestamps/1000.0;
               
%% Extract SWEEP

mask = strcmp(Type,'SWEEPCONVD');
mask2 = strcmp(Type,'SWEEPNEXT');
if(~isempty(mask))
    sweepIdx = find(strcmp(Type,'SWEEP'));
    fprintf('%d Sweeps found in study.\n', length(sweepIdx));
    
    % Combine sweeps
    combineTheseSweepIdxs = sweepIdx(1:end);
    allSweeps = cell(numel(combineTheseSweepIdxs),1);
    nSweepsTotal = 0;
    
    for i = 1:numel(combineTheseSweepIdxs)
        sweepIdx = combineTheseSweepIdxs(i);
        Time_SWEEP_Start = Time(sweepIdx);
    
        % get sweep parameters
        sweepSettings.nSweeps = x1(sweepIdx);
        sweepSettings.stepSize = x2(sweepIdx);
        sweepSettings.errorThresh = x3(sweepIdx);
        sweepSettings.imagingDuration = x4(sweepIdx);
        
        relevantIdx = find(mask);
        relevantIdx = relevantIdx(relevantIdx>sweepIdx);
        if(i < numel(combineTheseSweepIdxs))
            relevantIdx = relevantIdx(relevantIdx<combineTheseSweepIdxs(i+1));
        end
        Time_SWEEP_conv = Time(relevantIdx);
        SWEEP_convdControlCycle = CycleNum(relevantIdx);
        
        relevantIdx2 = find(mask2);
        relevantIdx2 = relevantIdx2(relevantIdx2>sweepIdx);
        Time_SWEEP_next = Time(relevantIdx2);
        SWEEP_psiCmd = rad2deg(x1(relevantIdx2));
        SWEEP_psiCmd = [SWEEP_psiCmd(1) - sweepSettings.stepSize; SWEEP_psiCmd];
        
        nSweeps = sweepSettings.nSweeps;
        
        % !!!!!!!!!!!!
        % nSweeps = nSweeps - 5;
        % !!!!!!!!!!!!
        
        % these better match
        if(nSweeps ~= length(relevantIdx))
            fprintf('nSweeps(%d) not equal to length(relevantIdx)=%d\n', nSweeps, length(relevantIdx));
        else
            fprintf('nSweeps(%d) is equal to length(relevantIdx)=%d\n', nSweeps, length(relevantIdx));
        end

        sweep(nSweeps) = struct('ImgFileNames',[],... % file name of image
            'ImgTimeStamps',[],... % time stamp of image
            'poseEM',[],... % interpolated pose
            'EMidx',[],... % index of closest EM reading
            'errorXYZ',[],...
            'errorPsi',[],... % angular error from desired pose
            'ECGcycleIdx',[],... % ECG cycle that the image belongs to
            'percentECG',[],... % location in the ECG cycle, value b/w 0.0 and 1.0
            'startIdx',[],...
            'startTime',[],...
            'endTime',[],...
            'controlCycle',[],...
            'targetPsi',[],...
            'errorPsiAtConvergence',[]);
        
        % Find sweep boundaries
        for j = 1:nSweeps
            % get start time
            sweepStartIdx = relevantIdx(j);
            sweepStartTime = Time(sweepStartIdx);
            
            % find first image in sweep
            imgT = imageTimestamps;
            imgT(imgT < sweepStartTime) = inf;
            [sweepStart_imageTime, sweepStart_imageIdx] = min(imgT);
            
            % get end time
            sweepEndTime = sweepStartTime + sweepSettings.imagingDuration/1000.;
            
            % find last image in sweep
            imgT = imageTimestamps;
            imgT(imgT > sweepEndTime) = 0;
            [sweepEnd_imageTime, sweepEnd_imageIdx] = max(imgT);
            
            % get a list of image filenames and timestamps
            imgTimesInSweep = imageTimestamps(sweepStart_imageIdx:sweepEnd_imageIdx);
            imgNamesInSweep = imageFileNames(sweepStart_imageIdx:sweepEnd_imageIdx);
            
            msk_ = dxyzpsi(:,1) == SWEEP_convdControlCycle(j);
            errorPsiAtConvergence = dxyzpsi(msk_, 6);
            
            sweep(j).ImgFileNames = imgNamesInSweep;
            sweep(j).ImgTimeStamps = imgTimesInSweep;
            sweep(j).startIdx = sweepStartIdx;
            sweep(j).startTime = sweepStartTime;
            sweep(j).endTime = sweepEndTime;
            sweep(j).controlCycle = SWEEP_convdControlCycle(j);
            sweep(j).targetPsi = SWEEP_psiCmd(j);
            sweep(j).errorPsiAtConvergence = errorPsiAtConvergence;
        end
        allSweeps{i} = sweep;
        nSweepsTotal = nSweepsTotal + nSweeps;
        clear sweep
    end

else
    disp('No SWEEPCONVD in file. Must be older format.');
end

sweep = [];
for i = 1:numel(combineTheseSweepIdxs)
    sweep = [sweep, allSweeps{i}];
end
nSweeps = numel(sweep);
assert(nSweepsTotal == nSweeps);

%% compare T_BB_CT and DXYZPSI
tS = [sweep(:).startTime];
tS = tS -Time_T_BB_CT(1);

nrm = sqrt(sum(squeeze(T_BB_CT(1:3,4,:)).^2,1));
plot(dxyzpsi(:,2)-Time_T_BB_CT(1), dxyzpsi(:,6)-mean(dxyzpsi(:,6)))
hold on
plot(Time_T_BB_CT - Time_T_BB_CT(1), nrm-mean(nrm))

plot(tS,2, '*')
legend('DXYZPSI','TBBCT','SWEEP')


%% Align image and EM timestamps

for i = 1:nSweeps
    nImgs = length(sweep(i).ImgTimeStamps);
    sweep(i).EMidx = zeros(nImgs,1);
    sweep(i).poseEM = zeros(4,4,nImgs);
    sweep(i).errorPsi = zeros(nImgs,1);
    sweep(i).errorXYZ = zeros(nImgs,3);
    for j = 1:nImgs
        % interpolate EM
        difft_ = Time_T_BB_CT - sweep(i).ImgTimeStamps(j);
        [~,minidx] = min(abs(difft_));
        if(difft_ > 0)
            t2idx = minidx - 1; % take previous point
            if(t2idx < 1)
                t2idx = 1;
            end
        else
            t2idx = minidx + 1; % take next point
            if(t2idx > length(Time_T_BB_CT))
                t2idx = length(Time_T_BB_CT);
            end
        end
        
        % interpolate EM accordingly
        pctgEM = abs(Time_T_BB_CT(minidx) - sweep(i).ImgTimeStamps(j))/abs(Time_T_BB_CT(minidx)-Time_T_BB_CT(t2idx));
        
        em1 = T_BB_CT(:,:,minidx);
        em2 = T_BB_CT(:,:,t2idx);
        emInt = trinterp(em1,em2,pctgEM);
        
        sweep(i).EMidx(j) = minidx;
        sweep(i).poseEM(:,:,j) = emInt;
        
        % get errorPsi
        currControlCycle = T_BB_CT_cycleNum(minidx);
        msk_ = dxyzpsi(:,1) == currControlCycle;
        sweep(i).errorPsi(j) = dxyzpsi(msk_,6);
        sweep(i).errorXYZ(j,:) = dxyzpsi(msk_,3:5);
        
        % TODO : Instead of using the world xyz, use the delta xyz to push
        % the images into place (because dxyz has the breathing info, but world doesn't)
    end
end

%% Load ECG

[ECG_time, ECGvoltage] = importECG([folder,filesep,study,'_ECG.txt']);
ECG_time = ECG_time/1000.0;
%% ECG Gating

minPeakDist = 60/100/2;
minPeakHei = 0.2;

[ECGpeakLocs, ECGpeakTime, avgHR, stdHR, phase] = ...
    ECGgating(ECGvoltage, ECG_time, minPeakDist, minPeakHei);

fprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);

%% Align ECG with images

% find out where in the heart cycle the image is taken from by looking
% at its alignment with the ECG signal
for i = 1:nSweeps
    nImgs = length(sweep(i).ImgTimeStamps);
    sweep(i).ECGcycleIdx = zeros(nImgs,1);
    sweep(i).percentECG = zeros(nImgs,1);
    for j = 1:nImgs
        diffImgECGtimes = sweep(i).ImgTimeStamps(j) - ECGpeakTime;
        diffImgECGtimes(diffImgECGtimes < 0) = Inf;
        [val_,ecgidx] = min(diffImgECGtimes); % gives us the first index
        if(isinf(val_))
            sweep(i).ECGcycleIdx(j) = NaN;
            continue;
        else
            ecgt_ = ECGpeakTime(ecgidx);
            difft_ = ecgt_ - sweep(i).ImgTimeStamps(j);
            ecgidx2 = ecgidx + 1;
            if(ecgidx2 > length(ECGpeakTime))
                sweep(i).ECGcycleIdx(j) = NaN;
                continue;
            end
            pctgECG = abs(difft_)/abs(ECGpeakTime(ecgidx) - ECGpeakTime(ecgidx2));
            sweep(i).ECGcycleIdx(j) = ecgidx;
            sweep(i).percentECG(j) = pctgECG;
        end
    end
end

%% Load crop mask

% cropMask = 'Sequoia_StarTech_Size 1 (largest)80mm';
% cropMask = 'Sequoia_StarTech_Size 1 (largest).mat';
load(['.',filesep,'cropMasks',filesep,cropMask,'.mat']);
% gives us a variable named cropSettings
cropSettings.mask_uint8 = uint8(cropSettings.mask);


%% Interpolate images - best ECG

n4Dframes = 15;

binCounts = zeros(nSweeps,n4Dframes);
temp = linspace(0,1,n4Dframes+1);
binMins = temp(1:end-1);
binMaxs = temp(2:end);
% find the best aligned ECG spot
for i = 1:nSweeps
    ecgs = sweep(i).percentECG(:);
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
    nSlicesToStitch = nSlicesToStitch4D(i);
    
    sub_goodSweepIdx = find(goodSweepIdx(:,i));
    
    for j = 1:nSlicesToStitch
        currSweepIdx = sub_goodSweepIdx(j);
        ecgs = sweep(currSweepIdx).percentECG';
        
        idxs = (ecgs < binMaxs(i)) & (ecgs >= binMins(i));
        listOfECGs{i} = [listOfECGs{i}, ecgs(idxs)];
    end
    phat(i,:) = mle(listOfECGs{i});
    fprintf(['Frame %d: Mean ECG phase is: %.3f ', char(177) ' %.3f\n'], i, phat(i,1), phat(i,2));
end

%% Stitch
interpCube = [];
dt = datestr(datetime('now'),'_yymmdd_HHMMss');
for k = 1:n4Dframes
    nSlicesToStitch = nSlicesToStitch4D(k);
    sub_goodSweepIdx = find(goodSweepIdx(:,k));
    
    for i = 1:nSlicesToStitch
        currSweepIdx = sub_goodSweepIdx(i);
        ecgs = sweep(currSweepIdx).percentECG';
        
        diff_ = abs(ecgs - phat(k,1));
        [~,imIdx] = min(diff_);
        
        fileName = sweep(currSweepIdx).ImgFileNames{imIdx};
        
        reportECG = sweep(currSweepIdx).percentECG(imIdx);
        fprintf('ECG: %.2f\n', reportECG);
        
        % Load image to memory
        fullName = [root,study,filesep,'images',filesep,fileName];
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
        
        stitch.originalEM{i} = sweep(currSweepIdx).poseEM(:,:,imIdx);
        
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
    [CdZeroed,interpCube,in3,stitch] = interpolateSlices4D(stitch, cropMask, 0, interpCube);
    %[CdZeroed] = interpolateSlicesGPU(stitch, cropMask, 'true');
    
    outfilePre = ['./volumes/volume_',study,dt,'_',num2str(k)];
    
    % save as .RAW
    volumeFileName = [outfilePre,'.raw'];
    saveStitched2RawFile(CdZeroed, volumeFileName)
    
    % save inhull mask as .mat
    save([outfilePre,'_inhull.mat'],'in3')
    
    % save as .mat
    save([outfilePre,'.mat'],'CdZeroed')
    
    % save volume size to text file
    [xn,yn,zn] = size(CdZeroed);
    
    fileID = fopen([outfilePre,'.txt'],'w');
    fprintf(fileID,'%d\t%d\t%d\n',xn,yn,zn);
    fclose(fileID);

    
    %stitches(k) = stitch;
end