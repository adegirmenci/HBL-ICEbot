function [images, EM_timeIdx, ECG_time, EM_timePctg] = selectFramesECG3(dataLoc, imLoc, ...
                                nPerSweep, EM_time, EM_epoch)
% selectFrames: ECG gating using the recorded ECG signal. Given an initial
% frame and the number of frames per sweep location, this function chooses 
% frames such that they are picked at the same location in the heart cycle. 
% This function is intended to be used with imageStitchingECG_v2.m.
% 'imLoc' variable is created in that script, and it is the directory
% location where the frame grabs are located.
%
% dataLoc : directory where image time stamps are located
% imLoc : directory where images are located
% nPerSweep : number of images acquired per sweep location
% initFrameNum : the index of the frame we want to start sampling from

if(nPerSweep < 1)
    error('nPerSweep should be positive')
elseif( ~isInt(nPerSweep) )
    error('nPerSweep should be an integer')
end

%images = dir([imLoc, '*.jpg']);

timeStampFiles = dir([dataLoc, '*AUTOSWEEP.txt']);
%timeStampFiles = dir([dataLoc, '*BUTTON.txt']);

if(isempty(timeStampFiles))
    error('timeStampFiles does not exist here')
elseif(length(timeStampFiles) ~= 1)
    error('Too many timeStampFiles')
end

% Import image file names and timestamps from folder
[FileName,time] = importImageTimestamps([dataLoc,timeStampFiles(1).name]);
% time = time;

nImages = length(time); % number of images found

nSweeps = nImages/nPerSweep;

if(~isInt(nSweeps))
    error('CRITICAL: nSweeps is not an integer!!!')
end

fprintf('%d sweeps with a total of %d images found.\n',nSweeps,nImages)

ECGfiles = dir([dataLoc, '*_ECG.txt']);

if(isempty(ECGfiles))
    error('ECGfiles does not exist here')
elseif(length(ECGfiles) ~= 1)
    error('Too many ECGfiles')
end

[ECG_time,ECGvoltage] = importECGdata2([dataLoc,ECGfiles(1).name]);
fprintf('ECG data found: %s.\n',ECGfiles(1).name)

% find peaks in ECG
figure
ECGvoltage = -ECGvoltage;
ECGvoltage(ECGvoltage < 0) = 0;
% findpeaks(ECGvoltage,'MinPeakDistance',2, 'MinPeakHeight',0.25);
% [~,ECGpeakLocs] = findpeaks(ECGvoltage,'MinPeakDistance',2, 'MinPeakHeight',0.25);
findpeaks(ECGvoltage,'MinPeakDistance',200, 'MinPeakHeight',0.20);
[~,ECGpeakLocs] = findpeaks(ECGvoltage,'MinPeakDistance',200, 'MinPeakHeight',0.20);
fprintf('Found %d peaks in the ECG signal.\n',length(ECGpeakLocs))
ECGpeakTime = ECG_time(ECGpeakLocs) + 0.445;

% Average heartbeat
diffECGpeaks = diff(ECGpeakTime);
meanECGpeaks = mean(diffECGpeaks);
stdECGpeaks = std(diffECGpeaks);
avgHR = 60./meanECGpeaks;
stdHR = 60.*stdECGpeaks/(meanECGpeaks+stdECGpeaks);
fprintf('Average heartrate: %.2f +- %.2f BPM\n', avgHR, stdHR);

% Find the first correspondange between image, ECG, and EM

% look at the current time interval for the sweep
% find the indices of EM and ECG data that fall between these times
% find the ECG peaks that exist here
% choose the ECG peak that is closest to the image time

images = struct('fileName',cell(nSweeps,1), 'time', 0);

timingError = zeros(nSweeps,1);

for i = 1:nSweeps
    begin_i = (i-1)*nPerSweep + 1;
    end_i = i*nPerSweep;
    
    % get image timestamps for the current sweep
    time_currSweep = time(begin_i:end_i);

    % find matching ECG times
    ECG_currSweepPeakTime = ECGpeakTime( (time_currSweep(1) <= ECGpeakTime) ...
                                    & (ECGpeakTime <= time_currSweep(end)) );
    
    if(isempty(ECG_currSweepPeakTime))
        fprintf('No ECG peaks detected in sweep %d of %d.\n',i,nSweeps)
        
%       images(i).fileName = [imLoc, FileName{currFrameIdx}(18:end)];
        images(i).fileName = [imLoc, FileName{begin_i}];
        images(i).time = time(begin_i);
        
        timingError(i) = time(end_i) - time(begin_i); % set time error to the whole interval
    else
        nPeaks = length(ECG_currSweepPeakTime);
        
        minError = zeros(nPeaks,1);
        minIdx = zeros(nPeaks,1);
        
        for j = 1:nPeaks
            ECGpeakTime_j = ECG_currSweepPeakTime(j);
            
            time_diff = time_currSweep - ECGpeakTime_j;
            [min_val, min_idx] = min(abs(time_diff));
            
            minError(j) = min_val;
            minIdx(j) = min_idx;
        end
        
        [min_val, min_idx] = min(minError);
        
%         currFrameIdx = min_idx + (i-1)*nPerSweep;
        currFrameIdx = minIdx(min_idx) + (i-1)*nPerSweep;
        
%       images(i).fileName = [imLoc, FileName{currFrameIdx}(18:end)];
        images(i).fileName = [imLoc, FileName{currFrameIdx}];
        images(i).time = time(currFrameIdx);
        
        timingError(i) = min_val;
            
    end
end

fprintf('Images extracted.\n')
fprintf('Min timing error is %.3f ms.\n', min(timingError)*1000.0)
fprintf('Max timing error is %.3f ms.\n', max(timingError)*1000.0)
fprintf('Mean timing error is %.3f ms.\n', mean(timingError)*1000.0)

% Select coresponding EM timestamps
EM_timeIdx = zeros(length(images),1);
EM_timeError = zeros(length(images),1);
EM_timePctg = zeros(length(images),1);

for i = 1:length(images)
    diff_t = EM_time - images(i).time;
    negT = diff_t;
    negT(negT > 0) = -Inf;
    [~,idx] = max(negT);
    
    pctg = abs(EM_time(idx) - images(i).time)/abs(EM_time(idx)-EM_time(idx+1));
    
    EM_timeIdx(i) = idx;
    EM_timePctg(i) = pctg;
    
    diff_t = abs(EM_time - images(i).time);
    [val,~] = min(diff_t);
    EM_timeError(i) = val;
end

fprintf('EM readings extracted.\n')
fprintf('Min timing error is %.3f ms.\n', min(EM_timeError)*1000.0)
fprintf('Max timing error is %.3f ms.\n', max(EM_timeError)*1000.0)
fprintf('Mean timing error is %.3f ms.\n', mean(EM_timeError)*1000.0)

end