function [allImgs, reorderedImgs, accurateImgs, nSweeps] = EM_ECG_matching(allImages, T_BB_CT, Time_T_BB_CT)
% EM_ECG_MATCHING: A part of ICEbot_imgProcGUI_v1
%      Written by Alperen Degirmenci
%      Harvard Biorobotics Lab
%      Last updated : 6/14/2017
%      Separates images into sweeps, aligns EM readings with images,
%      reorders images based on ECG cycle.
% See also: ICEBOT_IMGPROCGUI_V1

hWait = waitbar(0, 'Aligning EM, ECG, and US...');

% get EM readings
EMreadingTime = Time_T_BB_CT;
EMreading = T_BB_CT;

% Separate into sweeps
sweepInitIdx = logical(handles.finputs_.flag_updateorig_first_point);
sweepInitTimes = handles.finputs_.time(sweepInitIdx);
nSweeps = sum(handles.finputs_.flag_updateorig_first_point);
sweepBoundaryTimes = [sweepInitTimes, handles.finputs_.time(end)];
%plot
hold on
plot(sweepBoundaryTimes, zeros(length(sweepBoundaryTimes),1), 'x')
hold off

msg = sprintf('Found %d sweeps.', nSweeps);
waitbar(0.1, hWait, msg);

% go through all images and compute interped EM pose and pctg in ECG cycle
for i = 1:length(allImgs)
    % interpolate EM
    difft_ = EMreadingTime - allImgs(i).TimeStamp;
    [~,minidx] = min(abs(difft_));
    if(difft_ > 0)
        t2idx = minidx - 1; % take previous point
        if(t2idx < 1)
            t2idx = 1;
        end
    else
        t2idx = minidx + 1; % take next point
        if(t2idx > length(EMreadingTime))
            t2idx = length(EMreadingTime);
        end
    end
    
    % interpolate EM accordingly
    pctgEM = abs(EMreadingTime(minidx) - allImgs(i).TimeStamp)/abs(EMreadingTime(minidx)-EMreadingTime(t2idx));
    
    em1 = EMreading(:,:,minidx);
    em2 = EMreading(:,:,t2idx);
    emInt = trinterp(em1,em2,pctgEM);
    
    allImgs(i).EMidx = minidx;
    allImgs(i).poseEM = emInt;
    
    % find out where in the heart cycle the image is taken from by looking
    % at its alignment with the ECG signal
    diffImgECGtimes = allImgs(i).TimeStamp - handles.ECGpeakTime;
    diffImgECGtimes(diffImgECGtimes < 0) = Inf;
    [val_,ecgidx] = min(diffImgECGtimes); % gives us the first index
    if(isinf(val_))
        allImgs(i).ECGcycleIdx = NaN;
        continue;
    else
        ecgt_ = handles.ECGpeakTime(ecgidx);
        difft_ = ecgt_ - allImgs(i).TimeStamp;
        ecgidx2 = ecgidx + 1;
        if(ecgidx2 > length(handles.ECGpeakTime))
            allImgs(i).ECGcycleIdx = NaN;
            continue;
        end
        pctgECG = abs(difft_)/abs(handles.ECGpeakTime(ecgidx) - handles.ECGpeakTime(ecgidx2));
        allImgs(i).ECGcycleIdx = ecgidx;
        allImgs(i).percentECG = pctgECG;
    end
    
    % figure out to which sweep this image belongs
    diffImgSweepTime = allImgs(i).TimeStamp - sweepBoundaryTimes;
    diffImgSweepTime(diffImgSweepTime < 0) = Inf;
    [~,allImgs(i).sweepIdx] = min(diffImgSweepTime); % gives us the first index
    
    [tdif_,minidxPsi] = min(abs(handles.results_.time - allImgs(i).TimeStamp));
    if(tdif_ > 0.3)
        fprintf('Time error too large!\n');
    end
    allImgs(i).errorPsi = rad2deg(handles.results_.errorpsi(minidxPsi));
    
end
waitbar(0.5, hWait, 'Interpolated EM.');

% get rid of images that do not correspond to any ECG cycle
allImgs( isnan([allImgs(:).ECGcycleIdx]) ) = [];

waitbar(0.6, hWait, 'Cropped partial heart cycles.');

% reorder images in each sweep based on ECG timings
reorderedImgs = allImgs;
for i = 1:nSweeps
    inSweepIdx = find([allImgs(:).sweepIdx] == i);
    [~,sortedIdx] = sort([allImgs(inSweepIdx).percentECG]); % sort according to ECG
    notSorted = allImgs(inSweepIdx);
    reorderedImgs(inSweepIdx) = notSorted(sortedIdx); % reorder
end
waitbar(0.8, hWait, 'Reordered images based on ECG.');

% eliminate images with high error
errorThresh = 2.; %degrees
accurateImgs = reorderedImgs(abs([allImgs(:).errorPsi]) < errorThresh);
waitbar(1.0, hWait, 'Eliminated images with high error.');

% axes(handles.axes1)
% hold on
% plot(EMreadingTime(handles.alginedEMidx),zeros(length(handles.alginedEMidx),1)','^')
% hold off

close(hWait);

end